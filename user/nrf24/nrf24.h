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

/* Read-only live snapshots. Existing Nrf24Debug_t layout stays unchanged. */
typedef struct {
    uint32_t sample_count,sample_ms,sample_errors;
    uint32_t mismatch_mask;
    uint32_t tx_max_rt_count,tx_timeout_count,tx_bus_error_count;
    uint32_t tx_start_error_count,tx_payload_write_count,tx_ce_pulse_count;
    uint32_t last_tx_elapsed_ms;
    uint8_t valid;
    uint8_t config,en_aa,en_rxaddr,setup_aw,setup_retr,rf_ch,rf_setup;
    uint8_t status,observe_tx,rpd,fifo_status,rx_pw_p0,dynpd,feature;
    uint8_t rx_addr_p0[5],tx_addr[5];
    uint8_t ce_mcu_level,csn_mcu_level;
    /* Completion registers are captured BEFORE IRQ clear and TX FIFO flush. */
    uint8_t last_tx_result; /* 0=none,1=ACK,2=MAX_RT,3=10ms timeout,4=SPI/status error */
    uint8_t last_tx_status,last_tx_observe_tx,last_tx_fifo_status;
    uint8_t last_tx_extra_valid;
    uint8_t tx_fifo_after_load,tx_fifo_after_load_valid,tx_ce_high_seen;
} Nrf24RuntimeDebug_t;
extern volatile Nrf24RuntimeDebug_t g_nrf24_runtime;

HAL_StatusTypeDef NRF24_Init(void);
HAL_StatusTypeDef NRF24_ReadReg(uint8_t reg,uint8_t *value);
HAL_StatusTypeDef NRF24_WriteReg(uint8_t reg,uint8_t value);
HAL_StatusTypeDef NRF24_StartTx(const uint8_t payload[32]);
/* PollTx: 0=pending, 1=ACK success, -1=failed, 2=idle. */
int NRF24_PollTx(void);
/* Receive: 1=payload, 0=empty, -1=SPI/radio error. */
int NRF24_Receive(uint8_t payload[32]);
#endif