#ifndef NRF24_H
#define NRF24_H
#include "main.h"
#define NRF24_PAYLOAD_SIZE 32U
typedef struct {
    HAL_StatusTypeDef probe_status,spi_status,restore_status;
    uint8_t present,restore_attempted,status_byte,channel_before;
    uint8_t test_channel,channel_readback,channel_restored;
    uint8_t ready,role_rx,rf_setup,config;
    uint32_t tx_ok,tx_fail,rx_count,spi_errors;
} Nrf24Debug_t;
extern volatile Nrf24Debug_t g_nrf24_debug;
HAL_StatusTypeDef NRF24_Init(void);
HAL_StatusTypeDef NRF24_ReadReg(uint8_t reg,uint8_t *value);
HAL_StatusTypeDef NRF24_WriteReg(uint8_t reg,uint8_t value);
HAL_StatusTypeDef NRF24_StartTx(const uint8_t payload[32]);
/* PollTx: 0=pending, 1=ACK success, -1=failed, 2=idle. */
int NRF24_PollTx(void);
/* Receive: 1=payload, 0=empty, -1=SPI/radio error. */
int NRF24_Receive(uint8_t payload[32]);
#endif