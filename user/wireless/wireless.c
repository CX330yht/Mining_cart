#include "wireless.h"
#include "nrf24.h"
#include "cmsis_os.h"
static WirelessCommand_t s_published;
void Wireless_GetCommand(WirelessCommand_t *out)
{
    uint32_t mask=__get_PRIMASK();
    __disable_irq();
    *out=s_published;
    if(Link_Inhibited() || (uint32_t)(HAL_GetTick()-out->stamp_ms)>RADIO_TIMEOUT_MS)
        out->enabled=0;
    __set_PRIMASK(mask);
}
void Wireless_Task(void)
{
    Link_Init();
    /* NRF24_Init() is called once in main, before starting the scheduler. */
    for(;;) {
        uint8_t p[32];
        /* Only this task owns radio and link state; consumers see a snapshot. */
        for(unsigned i=0;i<3;i++) {
            int got=NRF24_Receive(p);
            if(got<0) { Wireless_RequestStop(LINK_RADIO); break; }
            if(!got) break;
            Link_Process(p,HAL_GetTick());
        }
        Link_Tick(HAL_GetTick());
        WirelessCommand_t next;
        Link_GetCommand(&next,HAL_GetTick());
        uint32_t mask=__get_PRIMASK(); __disable_irq();
        s_published=next;
        __set_PRIMASK(mask);
        osDelay(5);
    }
}