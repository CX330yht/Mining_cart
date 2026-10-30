#include "nrf24.h"
#include "spi.h"
#include <string.h>
#if defined(STM32F427xx)
#define RADIO_SPI hspi4
#define RADIO_RX 1U
#else
#define RADIO_SPI hspi1
#define RADIO_RX 0U
#endif
volatile Nrf24Debug_t g_nrf24_debug;
static uint8_t s_pending;
static uint32_t s_tx_start;
static const uint8_t s_address[5]={0xD2,0xA6,0xC3,0xE7,0x51};
static HAL_StatusTypeDef Xfer(uint8_t cmd,const uint8_t *out,uint8_t *in,uint8_t n)
{
    uint8_t tx[33]={0},rx[33]={0};
    if(n>32U) return HAL_ERROR;
    tx[0]=cmd;
    for(unsigned i=0;i<n;i++) tx[i+1]=out?out[i]:0xFFU;
    HAL_GPIO_WritePin(NRF_CSN_GPIO_Port,NRF_CSN_Pin,GPIO_PIN_RESET);
    HAL_StatusTypeDef st=HAL_SPI_TransmitReceive(&RADIO_SPI,tx,rx,n+1U,2U);
    HAL_GPIO_WritePin(NRF_CSN_GPIO_Port,NRF_CSN_Pin,GPIO_PIN_SET);
    g_nrf24_debug.spi_status=st;
    if(st==HAL_OK) {
        g_nrf24_debug.status_byte=rx[0];
        if(in) memcpy(in,rx+1,n);
    } else g_nrf24_debug.spi_errors++;
    return st;
}
HAL_StatusTypeDef NRF24_ReadReg(uint8_t reg,uint8_t *v)
{ return reg<=0x1F && v?Xfer(reg,0,v,1):HAL_ERROR; }
HAL_StatusTypeDef NRF24_WriteReg(uint8_t reg,uint8_t v)
{ return reg<=0x1F?Xfer((uint8_t)(0x20U|reg),&v,0,1):HAL_ERROR; }
static HAL_StatusTypeDef WriteChecked(uint8_t r,uint8_t v)
{
    uint8_t got=0;
    if(NRF24_WriteReg(r,v)!=HAL_OK || NRF24_ReadReg(r,&got)!=HAL_OK)
        return HAL_ERROR;
    return got==v?HAL_OK:HAL_ERROR;
}
HAL_StatusTypeDef NRF24_Init(void)
{
    g_nrf24_debug=(Nrf24Debug_t){0};
    g_nrf24_debug.probe_status=HAL_ERROR;
    g_nrf24_debug.role_rx=RADIO_RX; s_pending=0;
    HAL_GPIO_WritePin(NRF_CE_GPIO_Port,NRF_CE_Pin,GPIO_PIN_RESET);
    HAL_GPIO_WritePin(NRF_CSN_GPIO_Port,NRF_CSN_Pin,GPIO_PIN_SET);
    HAL_Delay(100);
    uint8_t old=0,got=0;
    if(NRF24_ReadReg(5,&old)!=HAL_OK || (old&0x80U)) return HAL_ERROR;
    g_nrf24_debug.channel_before=old;
    uint8_t test=old==76?42:76;
    g_nrf24_debug.test_channel=test;
    HAL_StatusTypeDef probe=WriteChecked(5,test);
    (void)NRF24_ReadReg(5,&got); g_nrf24_debug.channel_readback=got;
    g_nrf24_debug.restore_attempted=1;
    HAL_StatusTypeDef restore=WriteChecked(5,old);
    g_nrf24_debug.restore_status=restore;
    g_nrf24_debug.channel_restored=old;
    if(probe!=HAL_OK || restore!=HAL_OK) return HAL_ERROR;
    g_nrf24_debug.present=1;
    /* Fixed payload; no dynamic-payload/ACK-payload dependencies. */
    const uint8_t regs[][2]={
        {0x00,0x0C}, {0x01,0x01}, {0x02,0x01}, {0x03,0x03},
        {0x04,0x23}, {0x05,76}, {0x06,0x04}, {0x11,32},
        {0x1C,0}, {0x1D,0}
    };
    for(unsigned i=0;i<sizeof(regs)/sizeof(regs[0]);i++)
        if(WriteChecked(regs[i][0],regs[i][1])!=HAL_OK) return HAL_ERROR;
    if(Xfer(0x2A,s_address,0,5)!=HAL_OK ||
       Xfer(0x30,s_address,0,5)!=HAL_OK) return HAL_ERROR;
    uint8_t check[5];
    if(Xfer(0x0A,0,check,5)!=HAL_OK || memcmp(check,s_address,5) ||
       Xfer(0x10,0,check,5)!=HAL_OK || memcmp(check,s_address,5)) return HAL_ERROR;
    if(Xfer(0xE1,0,0,0)!=HAL_OK || Xfer(0xE2,0,0,0)!=HAL_OK ||
       NRF24_WriteReg(7,0x70)!=HAL_OK ||
       WriteChecked(0,(uint8_t)(RADIO_RX?0x0F:0x0E))!=HAL_OK) return HAL_ERROR;
    HAL_Delay(5);
    if(RADIO_RX) {
        HAL_GPIO_WritePin(NRF_CE_GPIO_Port,NRF_CE_Pin,GPIO_PIN_SET);
        HAL_Delay(1);
    }
    g_nrf24_debug.config=RADIO_RX?0x0F:0x0E;
    g_nrf24_debug.rf_setup=0x04;
    g_nrf24_debug.ready=1;
    g_nrf24_debug.probe_status=HAL_OK;
    return HAL_OK;
}
HAL_StatusTypeDef NRF24_StartTx(const uint8_t p[32])
{
    if(!g_nrf24_debug.ready || RADIO_RX || !p) return HAL_ERROR;
    if(s_pending) return HAL_BUSY;
    HAL_GPIO_WritePin(NRF_CE_GPIO_Port,NRF_CE_Pin,GPIO_PIN_RESET);
    if(NRF24_WriteReg(7,0x30)!=HAL_OK || Xfer(0xE1,0,0,0)!=HAL_OK ||
       Xfer(0xA0,p,0,32)!=HAL_OK) return HAL_ERROR;
    s_tx_start=HAL_GetTick(); s_pending=1;
    HAL_GPIO_WritePin(NRF_CE_GPIO_Port,NRF_CE_Pin,GPIO_PIN_SET);
    HAL_Delay(1); /* >=10us pulse; only R main loop blocks for this 1ms. */
    HAL_GPIO_WritePin(NRF_CE_GPIO_Port,NRF_CE_Pin,GPIO_PIN_RESET);
    return HAL_OK;
}
int NRF24_PollTx(void)
{
    if(!s_pending) return 2;
    uint8_t st=0;
    if(NRF24_ReadReg(7,&st)!=HAL_OK) st=0x10;
    if(st!=0xFFU && (st&0x20U)) {
        (void)NRF24_WriteReg(7,0x20); s_pending=0;
        g_nrf24_debug.tx_ok++; return 1;
    }
    if(st==0xFFU || (st&0x10U) || (uint32_t)(HAL_GetTick()-s_tx_start)>=10U) {
        (void)NRF24_WriteReg(7,0x30); (void)Xfer(0xE1,0,0,0);
        s_pending=0; g_nrf24_debug.tx_fail++; return -1;
    }
    return 0;
}
int NRF24_Receive(uint8_t p[32])
{
    if(!g_nrf24_debug.ready || !RADIO_RX) return -1;
    uint8_t fifo=0;
    if(NRF24_ReadReg(0x17,&fifo)!=HAL_OK || fifo==0xFFU) return -1;
    if(fifo&1U) return 0;
    if(Xfer(0x61,0,p,32)!=HAL_OK || NRF24_WriteReg(7,0x40)!=HAL_OK) return -1;
    g_nrf24_debug.rx_count++; return 1;
}