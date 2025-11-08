/** ========================================================================= *
 *
 * @file main.c
 * @date 20-05-2025
 * @author Maksym Tkachuk <max.r.tkachuk@gmail.com>
 *
 *  ========================================================================= */

/* Includes ================================================================= */
#include <sx1278.h>
#include <spi.h>
#include <log.h>
#include <stdlib.h>
#include <string.h>

/* Defines ================================================================== */
#define LOG_TAG MAIN

/* Macros =================================================================== */
#define WITH_SX1278(__handle, __spidev) \
    for (sx1278_t * __handle = __sx1278_init_static(__spidev); __handle; __sx1278_deinit_static(&__handle))

/* Exposed macros =========================================================== */
/* Enums ==================================================================== */
enum __sx1278_action {
  SX1278_INIT,
  SX1278_DEINIT,
};

/* Types ==================================================================== */
/* Variables ================================================================ */
const char ld_interp[] __attribute__((section(".interp"))) = LD_LOADER_PATH;

/* Private functions ======================================================== */
static void __sx1278_static_action(int action, ...) {
  static sx1278_t sx1278;
  static spi_t spi;

  va_list args;
  va_start(args, action);

  switch (action) {
    case SX1278_INIT: {
      sx1278_t ** sx1278_out = va_arg(args, sx1278_t **);

      spi_cfg_t spi_cfg;

      spi_cfg_default(&spi_cfg);
      spi_init(&spi, &spi_cfg, va_arg(args, const char *));

      sx1278_cfg_t sx1278_cfg = {.spi = &spi};

      sx1278_init(&sx1278, &sx1278_cfg);

      *sx1278_out = &sx1278;
      break;
    }

    case SX1278_DEINIT: {
      sx1278_deinit(&sx1278);
      spi_deinit(&spi);
      break;
    }

    default:
      break;
  }

  va_end(args);
}

static sx1278_t * __sx1278_init_static(const char * spidev) {
  sx1278_t * sx1278;

  __sx1278_static_action(SX1278_INIT, &sx1278, spidev);

  return sx1278;
}

static void __sx1278_deinit_static(sx1278_t ** sx1278) {
  __sx1278_static_action(SX1278_DEINIT);
  *sx1278 = NULL;
}

static void usage(const char * argv0) {
  log_printf(
    "Usage: %s [-l LEVEL] SPIDEV help|spitest|init|send|recv [TIMEOUT|BYTES]\n"
    "  help    - Shows this message\n"
    "  spitest - Tests SPI connection to sx1278 module\n"
    "  init    - Initializes sx1278 module\n"
    "  send    - Sends bytes via sx1278 module\n"
    "  recv    - Received a packet via a sx1278 module\n",
    argv0
  );
}

/* Shared functions ========================================================= */
int main(int argc, char ** argv) {
  size_t index = 1;

  if (argc > 2 && !strcmp(argv[1], "-l")) {
    log_level_t level = log_level_from_str(argv[2]);
    log_set_level(level);
    index = 3;
  }

  if (argc < index) {
    log_error("Insufficient arguments");
    usage(argv[0]);
    return 1;
  }

  const char * spidev = argv[index];

  if (!strcmp(argv[index+1], "spitest")) {
    spi_t spi;
    spi_cfg_t spi_cfg;

    spi_cfg_default(&spi_cfg);
    spi_init(&spi, &spi_cfg, spidev);

    // Version register
    uint8_t tx[2] = {0x42 & 0x7F, 0};
    uint8_t rx[2] = {0, 0};

    spi_transcieve(&spi, tx, rx, 2);

    log_info("Result: 0x%x 0x%x", rx[0], rx[1]);

    spi_deinit(&spi);
  } else if (!strcmp(argv[index+1], "init")) {
    WITH_SX1278(sx1278, spidev) {
      log_info("RA-02 Initialized");
    }
  } else if (!strcmp(argv[index+1], "send")) {
    uint8_t packet[SX1278_MAX_PACKET_SIZE] = {0};
    size_t size = 0;

    for (size_t i = index+2; i < argc; ++i) {
      packet[size++] = atoi(argv[i]);
    }

    error_t err = E_OK;

    WITH_SX1278(sx1278, spidev) {
      err = sx1278_send(sx1278, packet, size);

      if (err == E_OK) {
        log_info("Packet sent");
      } else {
        log_error("Failed to send packet: %s", error2str(err));
      }
    }

    return err == E_OK ? 0 : 1;
  } else if (!strcmp(argv[index+1], "recv")) {
    if (argc - index >= 2) {
      log_error("Expected TIMEOUT");
      usage(argv[0]);
      return 1;
    }

    TIMEOUT_CREATE(t, atoi(argv[index+2]));

    uint8_t packet[SX1278_MAX_PACKET_SIZE] = {0};
    size_t size = sizeof(packet);
    error_t err = E_OK;

    WITH_SX1278(sx1278, spidev) {
      err = sx1278_recv(sx1278, packet, &size, &t);
      if (err == E_OK) {
        log_printf("[%d]: ", size);
        for (size_t i = 0; i < size; ++i) {
          log_printf("%02x ", packet[i]);
        }
        log_printf("\n");
      } else {
        log_error("sx1278_recv: %s", error2str(err));
      }
    }

    return err == E_OK ? 0 : 1;
  } else {
    log_error("Unknown argument '%s'", argv[2]);
    usage(argv[0]);
    return 1;
  }

  return 0;
}

void __entry() {
#if __aarch64__
  // By SYSV ABI top of sp is [argc, argv, envp]
  asm volatile (
    "ldr x0, [sp]   \n" // Load argc
    "add x1, sp, #8 \n" // Load argv
    "bl main        \n" // Call main
    "mov x8, #93    \n" // exit() syscall number
    "svc #0         \n" // Execute syscall
  );
#else
  #error "Unsupported architecture"
#endif
}
