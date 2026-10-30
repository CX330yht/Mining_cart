#include "motor_control.h"
#include "encoder.h"
#include "pid.h"
#include "mecanum.h"
#include "wireless.h"
#include "cmsis_os.h"
#include <math.h>
volatile int32_t g_wheel_count[4];
volatile float g_wheel_rpm[4],g_wheel_rpm_filtered[4];
volatile float g_target_rpm[4],g_ramped_target_rpm[4],g_pid_command[4];
volatile float g_speed_kp[4]={.055f,.055f,.055f,.055f};
volatile float g_speed_ki[4]={.030f,.030f,.030f,.030f};
volatile float g_speed_kd[4]={0,0,0,0};
volatile uint32_t g_motor_enable,g_motor_mode=MOTOR_MODE_CHASSIS;
volatile MotorDebug_t g_motor_debug;
volatile float g_chassis_vx_mps,g_chassis_vy_mps,g_chassis_wz_radps,g_chassis_scale=1;
static const uint32_t channels[4]={TIM_CHANNEL_1,TIM_CHANNEL_3,TIM_CHANNEL_2,TIM_CHANNEL_4};
static TIM_HandleTypeDef *const timers[4]={&htim4,&htim8,&htim2,&htim5};
static const int8_t directions[4]={1,1,-1,1};
static float Ramp(float x,float y,float step)
{ return y>x+step?x+step:(y<x-step?x-step:y); }
static void Reset(PID_t p[4])
{
    TB6612_StopAll();
    for(unsigned i=0;i<4;i++) {
        PID_Reset(p+i);
        g_target_rpm[i]=g_ramped_target_rpm[i]=g_pid_command[i]=0;
    }
}
void MotorControl_Task(void)
{
    Encoder_t enc[4]; PID_t pid[4];
    HAL_StatusTypeDef st=TB6612_Init();
    g_motor_debug.start_status=st;
    if(st!=HAL_OK) Error_Handler();
    for(unsigned i=0;i<4;i++) {
        st=Encoder_Start(enc+i,timers[i],1560.0f,directions[i]);
        g_motor_debug.encoder_start_status[i]=st;
        if(st!=HAL_OK) { TB6612_StopAll(); Error_Handler(); }
        PID_Init(pid+i,g_speed_kp[i],g_speed_ki[i],0,TB6612_MAX_DUTY);
    }
    g_motor_debug.ready=1;
    uint32_t wake=osKernelSysTick(),last=wake,epoch=0;
    for(;;) {
        osDelayUntil(&wake,10);
        uint32_t now=osKernelSysTick(),ms=now-last; last=now;
        WirelessCommand_t cmd; Wireless_GetCommand(&cmd);
        float dt=ms*.001f,applied[4]={0},target[4]={0},drive[4]={0};
        if(ms && ms<=100) {
            float alpha=dt/(.05f+dt);
            for(unsigned i=0;i<4;i++) {
                Encoder_Update(enc+i,dt);
                g_wheel_count[i]=enc[i].total_count;
                g_wheel_rpm[i]=enc[i].speed_rpm;
                g_wheel_rpm_filtered[i]+=alpha*(g_wheel_rpm[i]-g_wheel_rpm_filtered[i]);
            }
        }
        uint8_t valid=ms>0 && ms<=100;
        float scale=1;
        if(cmd.enabled && valid)
            valid=Mecanum_Calculate(cmd.vx,cmd.vy,cmd.wz,LINK_WHEEL_MAX_RPM,target,&scale);
        for(unsigned i=0;i<4;i++)
            if(!isfinite(g_speed_kp[i]) || !isfinite(g_speed_ki[i]) ||
               !isfinite(g_speed_kd[i]) || g_speed_kp[i]<0 ||
               g_speed_ki[i]<0 || g_speed_kd[i]<0) valid=0;
        g_motor_enable=cmd.enabled && valid;
        g_chassis_vx_mps=g_motor_enable?cmd.vx:0;
        g_chassis_vy_mps=g_motor_enable?cmd.vy:0;
        g_chassis_wz_radps=g_motor_enable?cmd.wz:0;
        g_chassis_scale=scale;
        if(!g_motor_enable) {
            Reset(pid);
            if(!valid || g_motor_debug.stop_reason!=MOTOR_STOP_INVALID)
                g_motor_debug.stop_reason=valid?MOTOR_STOP_DISABLED:MOTOR_STOP_INVALID;
            if(!valid) Wireless_RequestStop(LINK_ACTUATOR);
        } else {
            if(epoch!=cmd.epoch) { Reset(pid); epoch=cmd.epoch; }
            for(unsigned i=0;i<4;i++) {
                g_target_rpm[i]=target[i];
                float old=g_ramped_target_rpm[i];
                float ramp=Ramp(old,target[i],100.0f*dt);
                if(old*ramp<0 || fabsf(ramp)<.001f) PID_Reset(pid+i);
                g_ramped_target_rpm[i]=ramp;
                pid[i].kp=g_speed_kp[i]; pid[i].ki=g_speed_ki[i]; pid[i].kd=g_speed_kd[i];
                drive[i]=PID_Calculate(pid+i,ramp,g_wheel_rpm_filtered[i],dt);
                if(drive[i]*ramp<0 || fabsf(ramp)<.001f) {
                    drive[i]=0; PID_Reset(pid+i);
                }
                if(!isfinite(drive[i])) valid=0;
            }
            if(!valid || TB6612_Apply(drive,applied)!=HAL_OK) {
                Reset(pid); g_motor_enable=0;
                g_motor_debug.stop_reason=MOTOR_STOP_INVALID;
                Wireless_RequestStop(LINK_ACTUATOR);
            } else g_motor_debug.stop_reason=MOTOR_STOP_NONE;
            for(unsigned i=0;i<4;i++) {
                g_pid_command[i]=drive[i];
                if(fabsf(drive[i])>.001f && fabsf(applied[i])<.00001f) PID_Reset(pid+i);
            }
        }
        uint32_t bits=0;
        for(unsigned i=0;i<4;i++) {
            if(fabsf(g_target_rpm[i])>.001f) bits|=1U<<i;
            g_motor_debug.applied_duty[i]=applied[i];
            g_motor_debug.pwm_compare[i]=__HAL_TIM_GET_COMPARE(&htim1,channels[i]);
        }
        g_motor_debug.active_mode=MOTOR_MODE_CHASSIS;
        g_motor_debug.speed_mask=bits; g_motor_debug.update_count++;
    }
}