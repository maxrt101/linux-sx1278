# SX1278 (RA01/RA-02) Driver for Linux

SX1278 (RA01/RA-02) driver for Linux that uses linux spidev API.
Provides C & Python APIs to send/receive data & configure the module.

### Usage

#### C API
```C
spi_t spi;
spi_cfg_t spi_cfg;

spi_cfg_default(&spi_cfg);
spi_init(&spi, &spi_cfg, "/dev/spidev0.0");

sx1278_t trx;
sx1278_cfg_t trx_cfg = {.spi = &spi};

sx1278_init(&trx, &trx_cfg);

// Send some bytes
uint8_t tx_data[] = {1, 2, 3, 4, 5};
sx1278_send(&sx1278, tx_data, sizeof(tx_data));

// Run receive for 5s
TIMEOUT_CREATE(timeout, 5000);
uint8_t rx_data[RA02_MAX_PACKET_SIZE] = {0};
size_t size = sizeof(rx_data);
sx1278_recv(sx1278, rx_data, &size, &timeout);

sx1278_deinit(&sx1278);
spi_deinit(&spi);
```

#### Python bindings
```python
import sx1278
ra02.__init__('./liblinux-sx1278.so')
spi = ra02.Spi('/dev/spidev0.0')
trx = ra02.Sx1278(spi)

# Send some bytes
trx.send([1, 2, 3, 4, 5])

# Run receive for 5s
trx.recv(sx1278.Timeout(5000))
```

### How to build
#### Prerequisites
 - CMake (at least 3.27)
 - Aarch64 linux gnu toolchain (`aarch64-unknown-linux-gnu`/`aarch64-linux-gnu-gcc`)

#### Steps
 - `cmake -B cmake-build-directory -S . -G "Unix Makefiles"`
 - `cmake --build cmake-build-directory`

### How to run
Warning: tested only on Raspberry PI, but theoretically work on any linux machine that has spidev interface.

#### For Raspberry PI:
| Name | Pin # | Pin name |
|:------:|:-------:|:--------:|
| MOSI | 19    |  GPIO12  |
| MISO | 21    |  GPIO13  |
| SCK  | 23    |  GPIO14  |
| NSS  | 24    |  GPIO11  |
| GND  | Any   |   GND    |
| 3.3V | 1     |   3V3    |

After connecting add `dtoverlay=spi0-hw-cs` at the end of `/boot/firmware/config.txt`.
Reboot the device.

To send a packet run `./linux_ra02.so /dev/spidev0.0 send 1 2 3 4 5`.
Where `1 2 3 4 5` are individual bytes of packet payload (in decimal).

To receive a packet run `./linux_ra02.so /dev/spidev0.0 recv 5000`.
Where `5000` is receiver timeout in milliseconds.
