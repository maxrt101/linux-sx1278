/** ========================================================================= *
 *
 * @file sx1278.h
 * @date 01-08-2024
 * @author Maksym Tkachuk <max.r.tkachuk@gmail.com>
 *
 * @brief Driver for LoRa RA01/RA-02 module based on sx1278
 *
 *  ========================================================================= */
#pragma once

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ================================================================= */
#include <stdint.h>
#include "spi.h"
#include "error.h"
#include "timeout.h"

/* Defines ================================================================== */
/**
 * Timeout in ms for TX_DONE flag to get up after TX was initiated
 */
#ifndef SX1278_SEND_IRQ_TIMEOUT
#define SX1278_SEND_IRQ_TIMEOUT 500
#endif

/**
 * Max packet payload in bytes
 */
#define SX1278_MAX_PACKET_SIZE 64

/* Macros =================================================================== */
/* Enums ==================================================================== */
/* Types ==================================================================== */
/**
 * RA-02 TRX driver config
 */
typedef struct {
  spi_t * spi;
} sx1278_cfg_t;

/**
 * RA-02 TRX driver context
 */
typedef struct {
  spi_t * spi;
  uint8_t irq_flags;
  int8_t  last_rssi;
} sx1278_t;

/* Variables ================================================================ */
/* Shared functions ========================================================= */
/**
 * Initializes RA02
 *
 * @param trx RA02 Context
 * @param cfg Valid RA02 Config
 */
error_t sx1278_init(sx1278_t * trx, sx1278_cfg_t * cfg);

/**
 * Deinitializes RA02
 *
 * @param trx RA02 Context
 */
error_t sx1278_deinit(sx1278_t * trx);

/**
 * Resets RA02
 *
 * @param trx RA02 Context
 */
error_t sx1278_reset(sx1278_t * trx);

/**
 * Transitions RA02 to sleep mode
 *
 * @param trx RA02 Context
 */
error_t sx1278_sleep(sx1278_t * trx);

/**
 * Sets frequency
 *
 * @param trx RA02 Context
 * @param khz Operating frequency in kHz
 */
error_t sx1278_set_freq(sx1278_t * trx, uint32_t khz);

/**
 * Gets output power
 *
 * @param trx RA02 Context
 * @param db Output
 */
error_t sx1278_get_power(sx1278_t * trx, uint8_t * db);

/**
 * Sets output power
 *
 * @param trx RA02 Context
 * @param db Power
 */
error_t sx1278_set_power(sx1278_t * trx, uint8_t db);

/**
 * Set sync word
 *
 * @param trx RA02 Context
 * @param sync_word Sync word
 */
error_t sx1278_set_sync_word(sx1278_t * trx, uint32_t sync_word);

/**
 * Set baudrate
 *
 * @param trx RA02 Context
 * @param baudrate Baudrate
 */
error_t sx1278_set_baudrate(sx1278_t * trx, uint32_t baudrate);

/**
 * Set bandwidth
 *
 * @param trx RA02 Context
 * @param bandwidth Bandwidth
 */
error_t sx1278_set_bandwidth(sx1278_t * trx, uint32_t bandwidth);

/**
 * Set preamble length
 *
 * @param trx RA02 Context
 * @param preamble Preamble length in bytes
 */
error_t sx1278_set_preamble(sx1278_t * trx, uint32_t preamble);

/**
 * Set Spreading Factor
 *
 * @param trx RA02 Context
 * @param sf Spreading Factor
 */
error_t sx1278_set_sf(sx1278_t * trx, uint8_t sf);

/**
 * Retrieves current RSSI
 *
 * @param trx RA02 Context
 * @param rssi Output
 */
error_t sx1278_get_rssi(sx1278_t * trx, int16_t * rssi);

/**
 * Send data over radio
 *
 * @param trx RA02 Context
 * @param buf Buffer to send
 * @param size Buffer size
 */
error_t sx1278_send(sx1278_t * trx, uint8_t * buf, size_t size);

/**
 * Receive data over radio
 *
 * @param trx RA02 Context
 * @param buf Buffer to receive into
 * @param size On input - pointer to variable with buffer size. On output - size of received data
 * @param timeout Timeout to wait for
 */
error_t sx1278_recv(sx1278_t * trx, uint8_t * buf, size_t * size, timeout_t * timeout);

#ifdef __cplusplus
}
#endif