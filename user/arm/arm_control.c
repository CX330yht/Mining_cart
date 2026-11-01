#include "arm_control.h"
#include "tim.h"
#include "cmsis_os.h"
#include "pid.h"
#include "wireless.h"
#include <math.h>
#define ARM_REVERSE_MS 500U
#define ARM_PWM_HARD_LIMIT 1.00f
volatile uint32_t g_arm_enable=0,g_arm_zero_request=0;
volatile int32_t g_arm_encoder_direction=1,g_arm_motor_direction=1;
volatile float g_arm_counts_per_joint_rev=12936.0f;
volatile float g_arm_target_deg=0,g_arm_min_deg=0,g_arm_max_deg=90;
volatile float g_arm_max_rpm=2.0f,g_arm_max_duty=ARM_PWM_HARD_LIMIT;
volatile float g_arm_pos_kp=.40f,g_arm_speed_kp=.50f,g_arm_speed_ki=1.50f;
volatile ArmDebug_t g_arm_debug;
static volatile uint32_t s_raw_count,s_invalid_edges;
static uint8_t s_previous_ab,s_active,s_output_running;
static uint32_t s_zero_raw,s_stop_tick;
static int8_t s_last_direction;
static float Clamp(float x, float limit)
{
    return x > limit ? limit : (x < -limit ? -limit : x);
}

static float TargetLowerBound(void)
{
    return fmaxf(0.0f,g_arm_min_deg);
}

static float LimitTargetAngle(float value)
{
    return fminf(g_arm_max_deg,fmaxf(TargetLowerBound(),value));
}

static uint8_t ReadAB(void)
{
    /* Both signals are on GPIOF; one IDR read gives a coherent snapshot. */
    uint32_t levels = GPIOF->IDR;
    return (uint8_t)(((levels & ARM_ENC_A_Pin) ? 2U : 0U) |
                     ((levels & ARM_ENC_B_Pin) ? 1U : 0U));
}

void Arm_EncoderIRQ(void)
{
    static const int8_t step[16] = {
        0, 1, -1, 0, -1, 0, 0, 1,
        1, 0, 0, -1, 0, -1, 1, 0
    };
    uint8_t next = ReadAB();
    uint8_t old = s_previous_ab;
    if ((old ^ next) == 3U) s_invalid_edges++;
    s_raw_count += (uint32_t)(int32_t)step[(old << 2U) | next];
    s_previous_ab = next;
}

static void BrakeChannel(uint32_t now)
{
    __HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_2, 0U);
    /* PWM=0 and IN1=IN2=1 explicitly short-brakes while STBY is high. */
    HAL_GPIO_WritePin(ARM_IN1_GPIO_Port, ARM_IN1_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(ARM_IN2_GPIO_Port, ARM_IN2_Pin, GPIO_PIN_SET);
    if (s_output_running) {
        s_stop_tick = now;
        s_output_running = 0U;
    }
    g_arm_debug.applied_duty = 0.0f;
    g_arm_debug.pwm_compare = 0U;
}

static void StandbyDrive(uint32_t now)
{
    HAL_GPIO_WritePin(ARM_STBY_GPIO_Port, ARM_STBY_Pin, GPIO_PIN_RESET);
    BrakeChannel(now);
    s_last_direction = 0;
}

static void Abort(uint32_t reason, uint32_t now, PID_t *speed)
{
    StandbyDrive(now);
    PID_Reset(speed);
    g_arm_enable = 0U;
    g_arm_target_deg = 0.0f;
    g_arm_debug.target_rpm = 0.0f;
    g_arm_debug.stop_reason = reason;
    s_active = 0U;
}

static float Apply(float duty, uint32_t now)
{
    /* Independent final protection even if a caller asks for more PWM. */
    duty = Clamp(duty, ARM_PWM_HARD_LIMIT);
    uint32_t period = __HAL_TIM_GET_AUTORELOAD(&htim9) + 1U;
    uint32_t magnitude = (uint32_t)(fabsf(duty) * (float)period + 0.5f);
    if (magnitude == 0U) {
        BrakeChannel(now);
        return 0.0f;
    }
    if (magnitude > period) magnitude = period;
    int8_t logical = duty >= 0.0f ? 1 : -1;
    int8_t physical = (int8_t)(logical * g_arm_motor_direction);
    if (s_last_direction && physical != s_last_direction) {
        BrakeChannel(now);
        if ((uint32_t)(now - s_stop_tick) < ARM_REVERSE_MS) return 0.0f;
    }
    HAL_GPIO_WritePin(ARM_IN1_GPIO_Port, ARM_IN1_Pin,
                      physical > 0 ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(ARM_IN2_GPIO_Port, ARM_IN2_Pin,
                      physical > 0 ? GPIO_PIN_RESET : GPIO_PIN_SET);
    uint32_t compare = magnitude;
    __HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_2, compare);
    s_output_running = 1U;
    s_last_direction = physical;
    g_arm_debug.pwm_compare = compare;
    g_arm_debug.applied_duty =
        (float)logical * (float)magnitude / (float)period;
    return g_arm_debug.applied_duty;
}


static uint8_t ConfigValid(void)
{
    return isfinite(g_arm_counts_per_joint_rev) && g_arm_counts_per_joint_rev>0 &&
        (g_arm_encoder_direction==1 || g_arm_encoder_direction==-1) &&
        (g_arm_motor_direction==1 || g_arm_motor_direction==-1) &&
        isfinite(g_arm_min_deg) && isfinite(g_arm_max_deg) &&
        g_arm_min_deg>=0 && g_arm_max_deg<=90 && g_arm_min_deg<g_arm_max_deg &&
        isfinite(g_arm_max_rpm) && g_arm_max_rpm>0 && g_arm_max_rpm<=2 &&
        isfinite(g_arm_max_duty) && g_arm_max_duty>0 && g_arm_max_duty<=ARM_PWM_HARD_LIMIT &&
        isfinite(g_arm_pos_kp) && g_arm_pos_kp>0 &&
        isfinite(g_arm_speed_kp) && g_arm_speed_kp>0 &&
        isfinite(g_arm_speed_ki) && g_arm_speed_ki>=0;
}
void ArmControl_Task(void)
{
    PID_t speed; PID_Init(&speed,.5f,1.5f,0,ARM_PWM_HARD_LIMIT);
    g_arm_debug=(ArmDebug_t){0}; s_previous_ab=ReadAB();
    uint32_t now=osKernelSysTick(); StandbyDrive(now);
    __HAL_DBGMCU_FREEZE_TIM9();
    __HAL_TIM_DISABLE_OCxPRELOAD(&htim9,TIM_CHANNEL_2);
    HAL_StatusTypeDef st=HAL_TIM_PWM_Start(&htim9,TIM_CHANNEL_2);
    g_arm_debug.start_status=st; g_arm_debug.ready=st==HAL_OK;
    uint32_t wake=now,last=now,previous=s_raw_count,epoch=0;
    uint32_t settle_start=now,wake_start=now;
    uint8_t settling=0,initial_zero_done=0;
    float filtered=0;
    for(;;) {
        osDelayUntil(&wake,10);
        now=osKernelSysTick(); uint32_t ms=now-last; last=now;
        WirelessCommand_t cmd; Wireless_GetCommand(&cmd);
        uint32_t raw=s_raw_count; int32_t delta=(int32_t)(raw-previous); previous=raw;
        g_arm_debug.raw_count=raw; g_arm_debug.encoder_count=(int32_t)raw;
        g_arm_debug.invalid_edges=s_invalid_edges; g_arm_debug.update_count++;
        if(!ConfigValid() || !ms || ms>100) {
            Abort(ARM_STOP_CONFIG,now,&speed);
            if(cmd.enabled) Wireless_RequestStop(LINK_ACTUATOR);
            continue;
        }
        /* cmd.enabled is published only after the receiver accepts a fresh,
           centered SW4 enable handshake. Calibrate once per A-board boot. */
        uint8_t first_enable_zero = cmd.enabled && !initial_zero_done &&
                                    g_arm_debug.ready;
        if(g_arm_zero_request || first_enable_zero) {
            g_arm_zero_request=0;
            /* Manual recalibration remains disabled-only; normal use needs none. */
            if(first_enable_zero || !cmd.enabled) {
                StandbyDrive(now); s_active=0;
                s_zero_raw=raw; g_arm_debug.zero_valid=1;
                g_arm_target_deg=0; filtered=0; delta=0;
                PID_Reset(&speed);
                settling=0; g_arm_debug.at_target=0;
                g_arm_debug.position_settle_ms=0;
                if(first_enable_zero) {
                    initial_zero_done=1;
                    g_arm_debug.auto_zero_count++;
                    g_arm_debug.auto_zero_epoch=cmd.epoch;
                }
            }
        }
        float dt=ms*.001f,dir=(float)g_arm_encoder_direction;
        float rpm=(float)delta*dir*60/(g_arm_counts_per_joint_rev*dt);
        filtered+=dt/(.10f+dt)*(rpm-filtered);
        float angle=(float)(int32_t)(raw-s_zero_raw)*dir*360/g_arm_counts_per_joint_rev;
        g_arm_debug.angle_deg=angle; g_arm_debug.rpm=filtered;
        if(!cmd.enabled || !g_arm_debug.zero_valid || !g_arm_debug.ready) {
            uint32_t reason=!g_arm_debug.zero_valid?ARM_STOP_NO_ZERO:ARM_STOP_DISABLED;
            if(s_active || !g_arm_debug.stop_reason ||
               g_arm_debug.stop_reason==ARM_STOP_NO_ZERO)
                Abort(reason,now,&speed);
            else StandbyDrive(now);
            if(cmd.enabled && !g_arm_debug.ready) Wireless_RequestStop(LINK_ACTUATOR);
            g_arm_debug.at_target=0; g_arm_debug.position_settle_ms=0; settling=0;
            continue;
        }
        uint8_t limit=HAL_GPIO_ReadPin(ARM_LIMIT_MIN_GPIO_Port,ARM_LIMIT_MIN_Pin)==GPIO_PIN_RESET ||
                      HAL_GPIO_ReadPin(ARM_LIMIT_MAX_GPIO_Port,ARM_LIMIT_MAX_Pin)==GPIO_PIN_RESET ||
                      angle>g_arm_max_deg;
        if(limit) {
            Abort(ARM_STOP_LIMIT,now,&speed); Wireless_RequestStop(LINK_ACTUATOR); continue;
        }
        if(!s_active || epoch!=cmd.epoch) {
            PID_Reset(&speed); StandbyDrive(now);
            g_arm_target_deg=LimitTargetAngle(angle);
            epoch=cmd.epoch; s_active=1; wake_start=now;
            HAL_GPIO_WritePin(ARM_STBY_GPIO_Port,ARM_STBY_Pin,GPIO_PIN_SET);
            settling=0;
        }
        g_arm_enable=1; g_arm_debug.stop_reason=ARM_STOP_NONE;
        /* Right Y is a joint position-target rate; neutral keeps this target. */
        /* Saturate the stored target itself: a held DOWN command at zero
           creates no hidden negative target or delay on the next UP command. */
        g_arm_target_deg=LimitTargetAngle(
            g_arm_target_deg+cmd.arm_axis*g_arm_max_rpm*6.0f*dt);
        float error=g_arm_target_deg-angle;
        g_arm_debug.position_target_deg=g_arm_target_deg;
        g_arm_debug.position_error_deg=error;
        uint8_t in_band=fabsf(error)<=1.0f;
        if(in_band && fabsf(filtered)<=.20f && fabsf(cmd.arm_axis)<.001f) {
            if(!settling) { settling=1; settle_start=now; }
            g_arm_debug.position_settle_ms=now-settle_start;
            g_arm_debug.at_target=g_arm_debug.position_settle_ms>=100;
        } else {
            settling=0; g_arm_debug.position_settle_ms=0; g_arm_debug.at_target=0;
        }
        float target=in_band?0:Clamp(g_arm_pos_kp*error,g_arm_max_rpm);
        g_arm_debug.target_rpm=target;
        speed.kp=g_arm_speed_kp; speed.ki=g_arm_speed_ki; speed.kd=0;
        speed.output_limit=g_arm_max_duty;
        if((uint32_t)(now-wake_start)<2U) continue;
        /* Integral may supply holding torque at zero commanded speed. */
        float duty=PID_Calculate(&speed,target,filtered,dt);
        if(!isfinite(duty)) {
            Abort(ARM_STOP_CONFIG,now,&speed); Wireless_RequestStop(LINK_ACTUATOR); continue;
        }
        /* The software zero boundary is directional, not a latched fault.
           Block only outward/downward torque at or below zero; retain upward
           correction and the ability to accept the next UP command. Physical
           limit-switch inputs above still stop the drive normally. */
        if(angle<=TargetLowerBound() && duty<0.0f) {
            duty=0.0f;
            PID_Reset(&speed);
        }
        float actual=Apply(duty,now);
        if(actual==0 && duty!=0) PID_Reset(&speed);
        /* Continuous remote control: no bench travel/duration or no-count timer. */
    }
}