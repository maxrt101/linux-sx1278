/** ========================================================================= *
 *
 * @file spi.c
 * @date 20-05-2025
 * @author Maksym Tkachuk <max.r.tkachuk@gmail.com>
 *
 *  ========================================================================= */

/* Includes ================================================================= */
#include <spi.h>
#include <assertion.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/types.h>
#include <linux/spi/spidev.h>

/* Defines ================================================================== */
/* Macros =================================================================== */
/* Exposed macros =========================================================== */
/* Enums ==================================================================== */
/* Types ==================================================================== */
/* Variables ================================================================ */
/* Private functions ======================================================== */
/* Shared functions ========================================================= */

error_t spi_cfg_default(spi_cfg_t * cfg) {
  ASSERT_RETURN(cfg, E_NULL);

  cfg->bits_per_word = 8;
  cfg->delay_us      = 0;
  cfg->speed         = 1000000;

  return E_OK;
}

error_t spi_init(spi_t * spi, spi_cfg_t * cfg, const char * dev) {
  ASSERT_RETURN(spi && cfg && dev, E_NULL);

  memcpy(&spi->cfg, cfg, sizeof(*cfg));
  spi->fd = open(dev, O_RDWR);

  return spi->fd < 0 ? E_FAILED : E_OK;
}

error_t spi_deinit(spi_t * spi) {
  ASSERT_RETURN(spi, E_NULL);

  close(spi->fd);
  spi->fd = 0;

  return E_OK;
}

error_t spi_transcieve(
  spi_t * spi,
  uint8_t * tx_buf,
  uint8_t * rx_buf,
  size_t size
) {
  ASSERT_RETURN(spi, E_NULL);

  struct spi_ioc_transfer trx = {0};

  trx.tx_buf        = (unsigned long long) tx_buf;
  trx.rx_buf        = (unsigned long long) rx_buf;
  trx.len           = size;
  trx.speed_hz      = spi->cfg.speed;
  trx.delay_usecs   = spi->cfg.delay_us;
  trx.bits_per_word = spi->cfg.bits_per_word;
  trx.cs_change     = 0;

  return ioctl(spi->fd, SPI_IOC_MESSAGE(1), &trx) == size ? E_OK : E_FAILED;
}
