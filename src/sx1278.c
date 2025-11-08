/** ========================================================================= *
 *
 * @file sx1278.h
 * @date 01-08-2024
 * @author Maksym Tkachuk <max.r.tkachuk@gmail.com>
 *
 * @brief TRX API Implementation for LoRa RA-02 module based on sx1278
 *
 *  ========================================================================= */

/* Includes ================================================================= */
#include <string.h>
#include "sx1278.h"

#include <stdio.h>
#include <unistd.h>

#include "sx1278_regs.h"
#include "assertion.h"
#include "util.h"
#include "log.h"
#include "spi.h"

/* Defines ================================================================== */
#define LOG_TAG sx1278

/** Internal constants */
#define SX1278_MAX_PA           20

#define SX1278_RSSI_HF_SUB (-157)
#define SX1278_RSSI_LF_SUB (-164)

/** Default internal trx configuration parameters */
#define SX1278_DEFAULT_CRC_RATE SX1278_CRC_RATE_4_7 /* CRC Rate */
#define SX1278_DEFAULT_SF       8                 /* Spreading Factor */
#define SX1278_DEFAULT_OCP_MA   120               /* OverCurrentProtection is mA */

/** Initial trx configuration parameters */
#define SX1278_INIT_FREQ        433000            /* Initial frequency */
#define SX1278_INIT_POWER       17                /* Initial output power */
#define SX1278_INIT_BANDWIDTH   125000            /* Initial bandwidth */
#define SX1278_INIT_PREAMBLE    10                /* Initial preamble size */

/* Macros =================================================================== */
/* Enums ==================================================================== */
/**
 * RA-02 OpModes
 */
typedef enum {
  SX1278_OP_MODE_SLEEP         = 0,
  SX1278_OP_MODE_STANDBY       = 1,
  SX1278_OP_MODE_TX            = 3,
  SX1278_OP_MODE_RX_CONTINUOUS = 5,
  SX1278_OP_MODE_RX_SINGLE     = 6,
} sx1278_op_mode_t;

/**
 * RA-02 CRC Rate values
 */
typedef enum {
  SX1278_CRC_RATE_4_5 = 1,
  SX1278_CRC_RATE_4_6 = 2,
  SX1278_CRC_RATE_4_7 = 3,
  SX1278_CRC_RATE_4_8 = 4,
} sx1278_crc_rate_t;

/**
 * RA-02 Power conversion table
 */
typedef enum {
  SX1278_POWER_11 = 0xF6,
  SX1278_POWER_14 = 0xF9,
  SX1278_POWER_17 = 0xFC,
  SX1278_POWER_20 = 0xFF,
} sx1278_power_t;

/**
 * RA-02 Bandwidth conversion table
 */
typedef enum {
  SX1278_BANDWIDTH_7_8_KHZ  = 0,
  SX1278_BANDWIDTH_10_4_KHZ = 1,
  SX1278_BANDWIDTH_15_6_KHZ = 2,
  SX1278_BANDWIDTH_20_8_KHZ = 3,
  SX1278_BANDWIDTH_31_2_KHZ = 4,
  SX1278_BANDWIDTH_41_7_KHZ = 5,
  SX1278_BANDWIDTH_62_5_KHZ = 6,
  SX1278_BANDWIDTH_125_KHZ  = 7,
  SX1278_BANDWIDTH_250_KHZ  = 8,
  SX1278_BANDWIDTH_500_KHZ  = 9,
} sx1278_bandwidth_t;

/* Types ==================================================================== */
/* Variables ================================================================ */
/**
 * RA-02 Power mapping table
 */
static util_map_range_table_entry_t sx1278_power_mapping_db[] = {
    {.from = 1,  .to = 10, .value = SX1278_POWER_11},
    {.from = 11, .to = 13, .value = SX1278_POWER_11},
    {.from = 14, .to = 16, .value = SX1278_POWER_14},
    {.from = 17, .to = 19, .value = SX1278_POWER_17},
    {.from = 20, .to = 99, .value = SX1278_POWER_20},
    /* Terminating entry */
    {.from = 0, .to = 0, .value = 0}
};

/**
 * RA-02 Bandwidth mapping table
 */
static util_map_range_table_entry_t sx1278_bandwidth_mapping_hz[] = {
    {.from = 1,      .to = 7799,   .value = SX1278_BANDWIDTH_7_8_KHZ},
    {.from = 7800,   .to = 10399,  .value = SX1278_BANDWIDTH_7_8_KHZ},
    {.from = 10400,  .to = 15599,  .value = SX1278_BANDWIDTH_10_4_KHZ},
    {.from = 15600,  .to = 20799,  .value = SX1278_BANDWIDTH_15_6_KHZ},
    {.from = 20800,  .to = 31199,  .value = SX1278_BANDWIDTH_20_8_KHZ},
    {.from = 31200,  .to = 41699,  .value = SX1278_BANDWIDTH_31_2_KHZ},
    {.from = 41700,  .to = 62499,  .value = SX1278_BANDWIDTH_41_7_KHZ},
    {.from = 62500,  .to = 124999, .value = SX1278_BANDWIDTH_62_5_KHZ},
    {.from = 125000, .to = 249000, .value = SX1278_BANDWIDTH_125_KHZ},
    {.from = 250000, .to = 499000, .value = SX1278_BANDWIDTH_250_KHZ},
    {.from = 500000, .to = 999999, .value = SX1278_BANDWIDTH_500_KHZ},
    /* Terminating entry */
    {.from = 0, .to = 0, .value = 0}
};

/* Private functions ======================================================== */
/**
 * Write value to register using SPI bus
 */
static error_t sx1278_write_reg(sx1278_t * trx, uint8_t reg, uint8_t value) {
  uint8_t data[2] = {reg | 0x80, value};

  error_t err = spi_transcieve(trx->spi, data, NULL, sizeof(data));

  log_debug("write_reg: %s reg=%02x val=%02x data={%02x, %02x}",
            error2str(err), reg, value, data[0], data[1]);

  return err;
}

/**
 * Read value from register using SPI bus
 */
static error_t sx1278_read_reg(sx1278_t * trx, uint8_t reg, uint8_t * value) {
  uint8_t tx_data[2] = {reg & 0x7F, 0};
  uint8_t rx_data[2] = {0};

  error_t err = spi_transcieve(
      trx->spi, tx_data, rx_data, 2
  );

  log_debug("read_reg: %s reg=%02x res={%02x, %02x}", error2str(err), reg, rx_data[0], rx_data[1]);

  if (err == E_OK) {
    *value = rx_data[1];
  }

  return err;
}

/**
 * Write buffer to register using SPI bus
 */
static error_t sx1278_write_burst(sx1278_t * trx, uint8_t addr, uint8_t * buf, size_t size) {
  uint8_t data = addr | 0x80;

  return spi_transcieve(trx->spi, buf, NULL, size + 1);
}

/**
 * Transitions RA-02 to selected OpMode
 */
static error_t sx1278_goto_op_mode(sx1278_t * trx, sx1278_op_mode_t mode) {
  return sx1278_write_reg(trx, SX1278_REG_OP_MODE, SX1278_OP_MODE_LORA_PREFIX | mode);
}

/**
 * Set OverCurrent protection
 *
 * @param[in] current Current threshold in mA
 */
static error_t sx1278_set_ocp(sx1278_t * trx, uint8_t current) {
  current = UTIL_CAP(current, 45, 240);

  if (current <= 120) {
    current = (current - 45) / 5;
  } else if (current <= 240) {
    current = (current - 30) / 10;
  }

  return sx1278_write_reg(trx, SX1278_REG_OCP, current + (1 << 5));
}

/**
 * Set CRC on/off
 */
static error_t sx1278_set_crc(sx1278_t * trx, bool on) {
  uint8_t data;
  ERROR_CHECK_RETURN(sx1278_read_reg(trx, SX1278_LORA_REG_MODEM_CFG_2, &data));
  // TODO: define modem cfg configuration values
  return sx1278_write_reg(trx, SX1278_LORA_REG_MODEM_CFG_2, (on << 2) | data);
}

/**
 * Set implicit header mode on/off
 */
static error_t sx1278_set_implicit_header_mode(sx1278_t * trx, bool on) {
  uint8_t data;
  ERROR_CHECK_RETURN(sx1278_read_reg(trx, SX1278_LORA_REG_MODEM_CFG_1, &data));
  return sx1278_write_reg(trx, SX1278_LORA_REG_MODEM_CFG_1, on ? (data | 1) : (data & ~(1)));
}

/**
 * Set RX Symbol timeout
 */
static error_t sx1278_set_rx_symbol_timeout(sx1278_t * trx, uint16_t value) {
  uint8_t data;
  ERROR_CHECK_RETURN(sx1278_read_reg(trx, SX1278_LORA_REG_MODEM_CFG_2, &data));
  ERROR_CHECK_RETURN(sx1278_write_reg(trx, SX1278_LORA_REG_MODEM_CFG_2, data | ((value >> 8) & 0x3)));
  return sx1278_write_reg(trx, SX1278_LORA_REG_SYMB_TIMEOUT_LSB, value & 0xFF); /* Set timeout to max */
}

/* Shared functions ========================================================= */
error_t sx1278_init(sx1278_t * trx, sx1278_cfg_t * cfg) {
  ASSERT_RETURN(trx && cfg && cfg->spi, E_NULL);

  trx->spi         = cfg->spi;
  trx->irq_flags   = 0;

  sx1278_reset(trx);

  log_info("Initializing SX1278...");

  uint8_t version;
  ERROR_CHECK_RETURN(sx1278_read_reg(trx, SX1278_REG_VERSION, &version));

  log_info("HW Version=0x%02x", version);

  ASSERT_RETURN(version == SX1278_HW_VERSION, E_NORESP);

  ERROR_CHECK_RETURN(sx1278_write_reg(trx, SX1278_REG_OP_MODE, SX1278_OP_MODE_SLEEP));

  /** Configure TRX */
  ERROR_CHECK_RETURN(sx1278_goto_op_mode(trx, SX1278_OP_MODE_SLEEP)); /* Transition RA02 to LoRa mode */
  ERROR_CHECK_RETURN(sx1278_set_freq(trx, SX1278_INIT_FREQ)); /* Set init frequency */
  ERROR_CHECK_RETURN(sx1278_set_power(trx, SX1278_INIT_POWER)); /* Set init output power */
  ERROR_CHECK_RETURN(sx1278_set_ocp(trx, SX1278_DEFAULT_OCP_MA)); /* Set OverCurrentProtection */
  ERROR_CHECK_RETURN(sx1278_write_reg(trx, SX1278_REG_LNA, 0x23)); /* Set LNA */
  ERROR_CHECK_RETURN(sx1278_write_reg(trx, SX1278_LORA_REG_MODEM_CFG_2, 1)); /* Reset Modem Cfg 1 */
  ERROR_CHECK_RETURN(sx1278_write_reg(trx, SX1278_LORA_REG_MODEM_CFG_2, 0)); /* Reset Modem Cfg 2 */
  ERROR_CHECK_RETURN(sx1278_set_implicit_header_mode(trx, false)); /* Set implicit header mode */
  ERROR_CHECK_RETURN(sx1278_set_rx_symbol_timeout(trx, 0x2FF)); /* Set RX Symbol Timeout */
  ERROR_CHECK_RETURN(sx1278_set_sf(trx, SX1278_DEFAULT_SF)); /* Set Spreading Factor */
  ERROR_CHECK_RETURN(sx1278_set_bandwidth(trx, SX1278_INIT_BANDWIDTH)); /* Set init bandwidth */
  ERROR_CHECK_RETURN(sx1278_set_preamble(trx, SX1278_INIT_PREAMBLE)); /* Set init preamble */

  ERROR_CHECK_RETURN(sx1278_goto_op_mode(trx, SX1278_OP_MODE_STANDBY));

  log_info("Initialization OK");

  return E_OK;
}

error_t sx1278_deinit(sx1278_t * trx) {
  ASSERT_RETURN(trx, E_NULL);

  log_info("Deinit");

  sx1278_goto_op_mode(trx, SX1278_OP_MODE_SLEEP);

  return E_OK;
}

error_t sx1278_reset(sx1278_t * trx) {
  ASSERT_RETURN(trx, E_NULL);

  log_info("Reset");

  // TODO: Proper reset
  sx1278_deinit(trx);
  return E_OK;
}

error_t sx1278_sleep(sx1278_t * trx) {
  ASSERT_RETURN(trx, E_NULL);

  return sx1278_goto_op_mode(trx, SX1278_OP_MODE_SLEEP);
}

error_t sx1278_set_freq(sx1278_t * trx, uint32_t khz) {
  ASSERT_RETURN(trx, E_NULL);

  log_info("Set freq: %d kHz", khz);

  uint32_t freq = ((khz/1000) * 524288) >> 5;

  ERROR_CHECK_RETURN(sx1278_write_reg(trx, SX1278_REG_FRF_MSB, freq >> 16));
  // TODO: Check delays
  usleep(5000);
  ERROR_CHECK_RETURN(sx1278_write_reg(trx, SX1278_REG_FRF_MID, freq >> 8));
  // TODO: Check delays
  usleep(5000);
  ERROR_CHECK_RETURN(sx1278_write_reg(trx, SX1278_REG_FRF_LSB, freq));
  // TODO: Check delays
  usleep(5000);

  return E_OK;
}

error_t sx1278_get_power(sx1278_t * trx, uint8_t * db) {
  ASSERT_RETURN(trx && db, E_NULL);

  ERROR_CHECK_RETURN(sx1278_read_reg(trx, SX1278_REG_PA_CFG, db));

  UTIL_MAP_RANGE_TABLE_REV(sx1278_power_mapping_db, *db, *db);

  return E_OK;
}

error_t sx1278_set_power(sx1278_t * trx, uint8_t db) {
  ASSERT_RETURN(trx, E_NULL);

  if (db < 1) { db = 1; }
  if (db > SX1278_MAX_PA) { db = SX1278_MAX_PA; }

  log_info("Set power: %d db", db);

  UTIL_MAP_RANGE_TABLE(sx1278_power_mapping_db, db, db);
  ERROR_CHECK_RETURN(sx1278_write_reg(trx, SX1278_REG_PA_CFG, db));
  // TODO: Check delays
  usleep(10000);
  return E_OK;
}

error_t sx1278_set_sync_word(sx1278_t * trx, uint32_t sync_word) {
  ASSERT_RETURN(trx, E_NULL);

  log_info("Set SyncWord: %x", sync_word);

  ERROR_CHECK_RETURN(sx1278_write_reg(trx, SX1278_LORA_REG_SYNC_WORD, sync_word));
  // TODO: Check delays
  usleep(10000);
  return E_OK;
}

error_t sx1278_set_baudrate(sx1278_t * trx, uint32_t baudrate) {
  ASSERT_RETURN(trx, E_NULL);

  log_info("Set baudrate: %d", baudrate);

  // TODO: Implement
  log_warn("sx1278_set_baudrate isn't implemented");

  return E_NOTIMPL;
}

error_t sx1278_set_bandwidth(sx1278_t * trx, uint32_t bandwidth) {
  ASSERT_RETURN(trx, E_NULL);

  UTIL_MAP_RANGE_TABLE(sx1278_bandwidth_mapping_hz, bandwidth, bandwidth);

  log_info("Set bandwidth: %d", bandwidth);

  uint8_t data;
  sx1278_read_reg(trx, SX1278_LORA_REG_MODEM_CFG_1, &data);

  data = (bandwidth << 4) | (SX1278_DEFAULT_CRC_RATE << 1) | data;
  return sx1278_write_reg(trx, SX1278_LORA_REG_MODEM_CFG_1, data);
}

error_t sx1278_set_preamble(sx1278_t * trx, uint32_t preamble) {
  ASSERT_RETURN(trx, E_NULL);

  log_info("Set preamble: %d", preamble);

  ERROR_CHECK_RETURN(sx1278_write_reg(trx, SX1278_LORA_REG_PREAMBLE_MSB, preamble >> 8));
  ERROR_CHECK_RETURN(sx1278_write_reg(trx, SX1278_LORA_REG_PREAMBLE_LSB, preamble));

  return E_OK;
}

error_t sx1278_set_sf(sx1278_t * trx, uint8_t sf) {
  ASSERT_RETURN(trx, E_NULL);

  log_info("Set SF: %d", sf);

  uint8_t data;
  sf = UTIL_CAP(sf, 6, 12);
  ERROR_CHECK_RETURN(sx1278_read_reg(trx, SX1278_LORA_REG_MODEM_CFG_2, &data));
  // TODO: define modem cfg configuration values
  return sx1278_write_reg(trx, SX1278_LORA_REG_MODEM_CFG_2, (sf << 4) | data);
}

error_t sx1278_get_rssi(sx1278_t * trx, int16_t * rssi) {
  ASSERT_RETURN(trx, E_NULL);

  *rssi = SX1278_RSSI_HF_SUB + trx->last_rssi;

  return E_OK;
}

error_t sx1278_poll_irq_flags(sx1278_t * trx) {
  sx1278_read_reg(trx, SX1278_LORA_REG_IRQ_FLAGS, &trx->irq_flags);
  sx1278_write_reg(trx, SX1278_LORA_REG_IRQ_FLAGS, trx->irq_flags);

  log_debug("DIO0 IRQ: 0x%02x", trx->irq_flags);

  return E_OK;
}

error_t sx1278_send(sx1278_t * trx, uint8_t * buf, size_t size) {
  ASSERT_RETURN(trx && buf, E_NULL);
  ASSERT_RETURN(size, E_INVAL);

#if USE_RA02_EXT_LOG_SEND_RECV
  char payload[256] = {0};
  size_t ofs = 0;

  for (size_t i = 0; i < size; ++i) {
    ofs += snprintf(payload + ofs, sizeof(payload) - ofs, "%02x ", buf[i]);
  }

  log_debug("send: [%d]: %s", size, payload);
#else
  log_debug("send: %d bytes", size);
#endif

  error_t err = E_OK;
  uint8_t data;

  trx->irq_flags = 0;

  ERROR_CHECK_RETURN(sx1278_goto_op_mode(trx, SX1278_OP_MODE_STANDBY));

  ERROR_CHECK_RETURN(sx1278_write_reg(trx, SX1278_REG_DIO_MAP_1,
                                    SX1278_LORA_MAP_DIO_0(SX1278_LORA_DIO_0_TX_DONE)));

  ERROR_CHECK_RETURN(sx1278_read_reg(trx, SX1278_LORA_REG_FIFO_TX_BASE_ADDR, &data));
  ERROR_CHECK_RETURN(sx1278_write_reg(trx, SX1278_LORA_REG_FIFO_ADDR_PTR, data));
  ERROR_CHECK_RETURN(sx1278_write_reg(trx, SX1278_LORA_REG_PAYLOAD_LEN, size));
  ERROR_CHECK_RETURN(sx1278_write_burst(trx, SX1278_REG_FIFO, buf, size));

  ERROR_CHECK_RETURN(sx1278_goto_op_mode(trx, SX1278_OP_MODE_TX));

  TIMEOUT_CREATE(t, SX1278_SEND_IRQ_TIMEOUT);

  while (1) {
    if (timeout_is_expired(&t)) {
      err = E_TIMEOUT;
      break;
    }

    sx1278_poll_irq_flags(trx);

    if (trx->irq_flags & SX1278_LORA_IRQ_FLAGS_TX_DONE) {
      break;
    }
  }

  ERROR_CHECK_RETURN(sx1278_goto_op_mode(trx, SX1278_OP_MODE_SLEEP));

  return err;
}

error_t sx1278_recv(sx1278_t * trx, uint8_t * buf, size_t * size, timeout_t * timeout) {
  ASSERT_RETURN(trx && buf && size && timeout, E_NULL);
  ASSERT_RETURN(*size, E_EMPTY);

  uint8_t data;

  trx->irq_flags = 0;

  ERROR_CHECK_RETURN(sx1278_goto_op_mode(trx, SX1278_OP_MODE_STANDBY));

  ERROR_CHECK_RETURN(sx1278_write_reg(trx, SX1278_REG_DIO_MAP_1,
                                    SX1278_LORA_MAP_DIO_0(SX1278_LORA_DIO_0_RX_DONE)));

  ERROR_CHECK_RETURN(sx1278_goto_op_mode(trx, SX1278_OP_MODE_RX_SINGLE));

  while (1) {
    if (timeout && timeout_is_expired(timeout)) {
      ERROR_CHECK_RETURN(sx1278_goto_op_mode(trx, SX1278_OP_MODE_SLEEP));
      return E_TIMEOUT;
    }

    sx1278_poll_irq_flags(trx);

    if (trx->irq_flags & SX1278_LORA_IRQ_FLAGS_VALID_HDR) {
      sx1278_read_reg(trx, SX1278_LORA_REG_LAST_PKT_RSSI_VAL, &trx->last_rssi);
    }

    if (trx->irq_flags & SX1278_LORA_IRQ_FLAGS_RX_DONE) {
      ERROR_CHECK_RETURN(sx1278_goto_op_mode(trx, SX1278_OP_MODE_STANDBY));

      /* Read received size */
      ERROR_CHECK_RETURN(sx1278_read_reg(trx, SX1278_LORA_REG_RX_NB_BYTES, &data));

      *size = data > *size ? *size : data;

      ERROR_CHECK_RETURN(sx1278_read_reg(trx, SX1278_LORA_REG_FIFO_RX_CURRENT_ADDR, &data));
      ERROR_CHECK_RETURN(sx1278_write_reg(trx, SX1278_LORA_REG_FIFO_ADDR_PTR, data));

      for (size_t i = 0; i < *size; ++i) {
        ERROR_CHECK_RETURN(sx1278_read_reg(trx, SX1278_REG_FIFO, &buf[i]));
      }

#if USE_RA02_EXT_LOG_SEND_RECV
      char payload[256] = {0};
      size_t ofs = 0;

      for (size_t i = 0; i < *size; ++i) {
        ofs += snprintf(payload + ofs, sizeof(payload) - ofs, "%02x ", buf[i]);
      }

      log_debug("recv: [%d]: %s", *size, payload);
#else
      log_debug("recv: %d bytes", *size);
#endif

      ERROR_CHECK_RETURN(sx1278_goto_op_mode(trx, SX1278_OP_MODE_SLEEP));
      return E_OK;
    }
  }
}
