#ifndef NRF24_H
#define NRF24_H

#include "main.h"

typedef struct
{
    HAL_StatusTypeDef probe_status;
    HAL_StatusTypeDef spi_status;
    HAL_StatusTypeDef restore_status;
    uint8_t present;
    uint8_t restore_attempted;
    uint8_t status_byte;
    uint8_t channel_before;
    uint8_t test_channel;
    uint8_t channel_readback;
    uint8_t channel_restored;
} Nrf24Debug_t;

extern volatile Nrf24Debug_t g_nrf24_debug;

HAL_StatusTypeDef NRF24_Init(void);
HAL_StatusTypeDef NRF24_ReadReg(uint8_t reg, uint8_t *value);
HAL_StatusTypeDef NRF24_WriteReg(uint8_t reg, uint8_t value);

#endif