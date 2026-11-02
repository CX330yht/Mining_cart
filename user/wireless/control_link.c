#include "control_link.h"
#include <math.h>
volatile WirelessDebug_t g_wireless_debug;
volatile uint32_t g_wireless_enable=1U;
static WirelessCommand_t s_command;
static uint32_t s_sequence,s_token,s_base_token;
static uint8_t s_have_seq,s_ready,s_manual_blocked;
static volatile uint32_t s_stop_request;
static void Lock(uint32_t reason)
{
    s_command.enabled=0; s_ready=0; s_base_token=s_token;
    s_command.arm_auto_active=0; s_command.arm_preset_64=0;
    g_wireless_debug.arm_auto_active=0; g_wireless_debug.arm_preset_64=0;
    g_wireless_debug.armed=0; g_wireless_debug.rearm_ready=0;
    g_wireless_debug.stop_reason=reason;
}
void Link_Init(void)
{
    memset(&s_command,0,sizeof(s_command));
    g_wireless_debug=(WirelessDebug_t){0};
    s_sequence=s_token=s_base_token=0; s_have_seq=s_ready=s_manual_blocked=0;
    s_stop_request=0; g_wireless_enable=1;
    Lock(LINK_BOOT);
}
void Wireless_RequestStop(uint32_t why)
{ s_stop_request=why?why:LINK_ACTUATOR; }
uint8_t Link_Inhibited(void)
{ return s_stop_request!=0U || !g_wireless_enable; }
void Link_Tick(uint32_t now)
{
    if(s_stop_request) {
        uint32_t why=s_stop_request; s_stop_request=0;
        Lock(why);
    }
    if(!g_wireless_enable) {
        Lock(LINK_MANUAL); s_manual_blocked=1;
    } else if(s_manual_blocked) {
        Lock(LINK_MANUAL); s_manual_blocked=0;
    }
    if(s_have_seq && (uint32_t)(now-s_command.stamp_ms)>RADIO_TIMEOUT_MS) {
        Lock(LINK_TIMEOUT); s_have_seq=0;
        g_wireless_debug.link_ok=0;
    }
}
void Link_Process(const uint8_t data[32],uint32_t now)
{
    Link_Tick(now);
    RadioPacket_t p;
    if(!RadioDecode(data,&p)) { g_wireless_debug.bad_packets++; return; }
    /* Emergency stop is honored even if its sequence is stale. */
    if((p.flags&RADIO_FLAG_ESTOP) || (p.keys&0x20U)) Lock(LINK_ESTOP);
    if(s_have_seq) {
        uint32_t delta=p.sequence-s_sequence;
        if(!delta || delta>=0x80000000U) { g_wireless_debug.duplicates++; return; }
    } else {
        s_base_token=p.arm_token; s_ready=0;
    }
    s_have_seq=1; s_sequence=p.sequence; s_token=p.arm_token;
    s_command.stamp_ms=now;
    g_wireless_debug.last_rx_ms=now; g_wireless_debug.last_sequence=p.sequence;
    g_wireless_debug.arm_token=p.arm_token;
    g_wireless_debug.good_packets++; g_wireless_debug.link_ok=1;
    if(!g_wireless_enable || s_manual_blocked ||
       (p.flags&RADIO_FLAG_ESTOP) || (p.keys&0x20U)) {
        Lock(!g_wireless_enable?LINK_MANUAL:LINK_ESTOP); return;
    }
    if(!(p.flags&RADIO_FLAG_FRESH)) { Lock(LINK_INPUT); return; }
    if(!(p.flags&RADIO_FLAG_ENABLED) && s_command.enabled) Lock(LINK_DISABLED);
    if(!s_command.enabled) {
        if(!RadioCentered(&p)) {
            s_ready=0; s_base_token=p.arm_token;
        } else if(!(p.keys&RADIO_ENABLE_KEY)) {
            s_ready=1; s_base_token=p.arm_token;
        } else if(s_ready && p.arm_token!=s_base_token &&
                  (p.flags&RADIO_FLAG_ENABLED)) {
            s_command.enabled=1; s_command.epoch++;
            s_ready=0; g_wireless_debug.stop_reason=0;
        }
    }
    g_wireless_debug.armed=s_command.enabled;
    g_wireless_debug.rearm_ready=s_ready;
    g_wireless_debug.epoch=s_command.epoch;
    if(!s_command.enabled) return;
    static const float gears[3]={0.30f,0.60f,1.0f};
    float scale=gears[p.gear];
    float linear=LINK_WHEEL_MAX_RPM*(2.0f*3.14159265359f*0.030f)/60.0f;
    s_command.vx=(float)p.axis[1]*0.001f*linear*scale;
    s_command.vy=-(float)p.axis[0]*0.001f*linear*scale;
    int turn=((p.keys&1U)?1:0)-((p.keys&2U)?1:0);
    s_command.wz=(float)turn*linear/0.32725f*scale;
    s_command.arm_axis=(float)p.axis[3]*0.001f;
    s_command.arm_auto_active=(p.flags&RADIO_FLAG_ARM_AUTO)?1U:0U;
    s_command.arm_preset_64=(p.flags&RADIO_FLAG_ARM_64)?1U:0U;
    g_wireless_debug.arm_auto_active=s_command.arm_auto_active;
    g_wireless_debug.arm_preset_64=s_command.arm_preset_64;
    s_command.gear=p.gear;
    for(unsigned i=0;i<4;i++) s_command.servo_deg[i]=p.servo_ddeg[i]*0.1f;
}
void Link_GetCommand(WirelessCommand_t *out,uint32_t now)
{
    *out=s_command;
    /* Consumers check age themselves even if the radio task stops running. */
    if(!g_wireless_enable || s_stop_request || !s_have_seq ||
       (uint32_t)(now-out->stamp_ms)>RADIO_TIMEOUT_MS) out->enabled=0;
}