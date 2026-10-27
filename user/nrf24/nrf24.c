#include "nrf24.h"
#include "spi.h"

#define NRF_CMD_R_REGISTER 0x00U
#define NRF_CMD_W_REGISTER 0x20U
#define NRF_REG_MASK       0x1FU
#define NRF_REG_RF_CH      0x05U
#define NRF_SPI_TIMEOUT_MS 2U

volatile Nrf24Debug_t g_nrf24_debug;

static HAL_StatusTypeDef Nrf24_Exchange(uint8_t command, uint8_t data,
                                        uint8_t *reply)
{
    uint8_t tx[2] = {command, data};
    uint8_t rx[2] = {0};

    HAL_GPIO_WritePin(NRF_CSN_GPIO_Port, NRF_CSN_Pin, GPIO_PIN_RESET);
    HAL_StatusTypeDef status = HAL_SPI_TransmitReceive(
        &hspi4, tx, rx, 2U, NRF_SPI_TIMEOUT_MS);
    /* Release CSN even when the HAL operation fails. */
    HAL_GPIO_WritePin(NRF_CSN_GPIO_Port, NRF_CSN_Pin, GPIO_PIN_SET);

    g_nrf24_debug.spi_status = status;
    if (status == HAL_OK)
    {
        g_nrf24_debug.status_byte = rx[0];
        if (reply != NULL)
        {
            *reply = rx[1];
        }
    }
    return status;
}

HAL_StatusTypeDef NRF24_ReadReg(uint8_t reg, uint8_t *value)
{
    if (reg > NRF_REG_MASK || value == NULL)
    {
        return HAL_ERROR;
    }
    return Nrf24_Exchange((uint8_t)(NRF_CMD_R_REGISTER | reg), 0xFFU, value);
}

HAL_StatusTypeDef NRF24_WriteReg(uint8_t reg, uint8_t value)
{
    if (reg > NRF_REG_MASK)
    {
        return HAL_ERROR;
    }
    /* Configuration writes require power-down or standby mode. */
    return Nrf24_Exchange((uint8_t)(NRF_CMD_W_REGISTER | reg), value, NULL);
}

static HAL_StatusTypeDef Nrf24_ProbeChannel(void)
{
    uint8_t original = 0U;
    uint8_t observed = 0U;
    uint8_t restored = 0U;

    HAL_StatusTypeDef status = NRF24_ReadReg(NRF_REG_RF_CH, &original);
    g_nrf24_debug.channel_before = original;
    if (status != HAL_OK)
    {
        return status;
    }
    /* RF_CH bit 7 is reserved zero; this also rejects a stuck-high MISO. */
    if ((original & 0x80U) != 0U)
    {
        return HAL_ERROR;
    }

    /* Always change the value, including when the old channel is already 76. */
    uint8_t test = (original == 76U) ? 42U : 76U;
    g_nrf24_debug.test_channel = test;

    status = NRF24_WriteReg(NRF_REG_RF_CH, test);
    if (status == HAL_OK)
    {
        status = NRF24_ReadReg(NRF_REG_RF_CH, &observed);
        g_nrf24_debug.channel_readback = observed;
        if (status == HAL_OK && observed != test)
        {
            status = HAL_ERROR;
        }
    }

    /* A failed write can still have reached the chip; attempt restoration. */
    g_nrf24_debug.restore_attempted = 1U;
    HAL_StatusTypeDef restore = NRF24_WriteReg(NRF_REG_RF_CH, original);
    if (restore == HAL_OK)
    {
        restore = NRF24_ReadReg(NRF_REG_RF_CH, &restored);
        g_nrf24_debug.channel_restored = restored;
        if (restore == HAL_OK && restored != original)
        {
            restore = HAL_ERROR;
        }
    }
    g_nrf24_debug.restore_status = restore;

    return (status == HAL_OK) ? restore : status;
}

HAL_StatusTypeDef NRF24_Init(void)
{
    g_nrf24_debug = (Nrf24Debug_t){0};
    g_nrf24_debug.probe_status = HAL_ERROR;

    HAL_GPIO_WritePin(NRF_CE_GPIO_Port, NRF_CE_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(NRF_CSN_GPIO_Port, NRF_CSN_Pin, GPIO_PIN_SET);

    /* One-time startup wait for the radio's power-on reset. */
    HAL_Delay(100U);

    HAL_StatusTypeDef status = Nrf24_ProbeChannel();
    g_nrf24_debug.probe_status = status;
    g_nrf24_debug.present = (status == HAL_OK) ? 1U : 0U;
    return status;
}