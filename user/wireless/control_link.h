#ifndef CONTROL_LINK_H
#define CONTROL_LINK_H
#include "radio_protocol.h"
#define LINK_WHEEL_MAX_RPM 40.0f
enum { LINK_BOOT=1, LINK_TIMEOUT=2, LINK_ESTOP=3, LINK_INPUT=4,
       LINK_DISABLED=5, LINK_ACTUATOR=6, LINK_RADIO=7, LINK_MANUAL=8 };
typedef struct {
    uint32_t stamp_ms,epoch;
    uint8_t enabled,gear;
    float vx,vy,wz,arm_axis;
    float servo_deg[4];
} WirelessCommand_t;
typedef struct {
    uint32_t good_packets,bad_packets,duplicates,last_sequence,last_rx_ms;
    uint32_t epoch,stop_reason,arm_token;
    uint8_t link_ok,armed,rearm_ready;
} WirelessDebug_t;
extern volatile WirelessDebug_t g_wireless_debug;
/* 0 immediately inhibits all consumers; restore to 1 then press SW4 again. */
extern volatile uint32_t g_wireless_enable;
void Link_Init(void);
void Link_Process(const uint8_t p[32],uint32_t now);
void Link_Tick(uint32_t now);
void Link_GetCommand(WirelessCommand_t *out,uint32_t now);
void Wireless_RequestStop(uint32_t reason);
uint8_t Link_Inhibited(void);
#endif