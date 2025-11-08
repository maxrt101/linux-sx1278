# =========================================================================
#
# :file: sx1278.py
# :date: 20-05-2025
# :author: Maksym Tkachuk <max.r.tkachuk@gmail.com>
#
# LinuxRA02 Python bindings to sx1278 shared library
#
# Provides API for RA02 RF module control, SPI & Timeout abstractions
# Basically a thin FFI interface to C driver implementation
#
# Example:
#   # Initialize the library, SPI & RA02 module
#   import sx1278
#   sx1278.__init__('./linux_sx1278.so')
#   spi = sx1278.Spi('/dev/spidev0.0')
#   rf = sx1278.Sx1278(spi)
#
#   # Send some bytes
#   rf.send(bytes([1, 2, 3, 4, 5]))
#
#   # Run receive for 5s
#   rf.recv(sx1278.Timeout(5000))
#
# =========================================================================

import ctypes


class DynamicLibrary:
    """
    Dynamic library wrapper that throws exceptions when it's uninitialized
    """

    def __init__(self, dynlib_path: str = None):
        """
        Initialize DynamicLibrary. Loads dynlib if not None

        :param dynlib_path: Path to shared library object
        """

        self.dynlib = None

        if dynlib_path:
            self.load(dynlib_path)


    def load(self, dynlib_path: str):
        """
        Loads dynlib using ctypes

        :param dynlib_path: Path to shared library object
        """

        self.dynlib = ctypes.cdll.LoadLibrary(dynlib_path)

    def __getattr__(self, item):
        """
        Retrieves symbol from dynlib, if available, otherwise throws exception

        :param item: Symbol name
        :return: Symbol (ctypes wrapper)
        """

        if self.dynlib:
            return getattr(self.dynlib, item)
        else:
            raise LibraryUninitializedException('RA02 DynamicLibrary in uninitialized. Call sx1278.__init__')



# Global (for module) handle to opened sx1278 shared library
RA02_DYNLIB = DynamicLibrary()


# Python exceptions
class InvalidErrorCodeException(Exception): ...
class LibraryUninitializedException(Exception): ...


# Exceptions mirrored from error codes in error.h
class FailedException(Exception): ...
class AssertException(Exception): ...
class NullException(Exception): ...
class InvalidException(Exception): ...
class NotImplementedException(Exception): ...
class TimeoutException(Exception): ...
class NoResponseException(Exception): ...
class OverflowException(Exception): ...
class UnderflowException(Exception): ...
class AgainException(Exception): ...
class DoneException(Exception): ...
class CorruptException(Exception): ...
class BusyException(Exception): ...
class NotFoundException(Exception): ...
class CancelledException(Exception): ...
class EmptyException(Exception): ...
class NoMemoryException(Exception): ...
class OutOfBoundsException(Exception): ...

# Port of error_t from error.h
EXCEPTIONS = {
    1: FailedException,
    2: AssertException,
    3: NullException,
    4: InvalidException,
    5: NotImplementedException,
    6: TimeoutException,
    7: NoResponseException,
    8: OverflowException,
    9: UnderflowException,
    10: AgainException,
    11: DoneException,
    12: CorruptException,
    13: BusyException,
    14: NotFoundException,
    15: CancelledException,
    16: EmptyException,
    17: NoMemoryException,
    18: OutOfBoundsException,
}

def error_check(err: int):
    """
    Checks error code, returned by sx1278.h/spi.h APIs
    and throws a mirroring exception, if error code is not 0 (E_OK)

    :param err: Value from error_t enum
    """
    if err != 0 and err != None:
        if err in EXCEPTIONS:
            raise EXCEPTIONS[err]
        else:
            raise InvalidErrorCodeException(f'{err}')


class spi_cfg_t(ctypes.Structure):
    """
    Defines SPI config from spi.h
    """
    _fields_ = [
        ('speed', ctypes.c_uint32),
        ('delay_us', ctypes.c_uint16),
        ('bits_per_word', ctypes.c_uint8),
    ]

class spi_t(ctypes.Structure):
    """
    Defines SPI context from spi.h
    """
    _fields_ = [
        ('cfg', spi_cfg_t),
        ('fd', ctypes.c_int),
    ]

class Spi:
    """
    Encapsulates spi_t and spi_* APIs from spi.h
    """

    def __init__(self, spidev: str = None):
        """
        Initializes SPI

        :param spidev: String, that represents a path to a valid linux spidev file (or None if it's not needed to be initialized)
        """

        self.spi = spi_t()

        if spidev:
            self.init(spidev)

    def __del__(self):
        """
        Deinitializes SPI
        """

        self.deinit()

    def init(self, spidev: str):
        """
        Initializes SPI

        :param spidev: String, that represents a path to a valid linux spidev file (or None if it's not needed to be initialized)
        """

        cfg = spi_cfg_t()

        error_check(RA02_DYNLIB.spi_cfg_default(ctypes.byref(cfg)))
        error_check(RA02_DYNLIB.spi_init(ctypes.byref(self.spi), ctypes.byref(cfg), spidev.encode('utf-8')))

    def deinit(self):
        """
        Deinitializes SPI
        """

        if self.spi.fd != 0:
            error_check(RA02_DYNLIB.spi_deinit(ctypes.byref(self.spi)))

    def transceive(self, data: list[int]) -> list[int]:
        """
        Send/Receive data over SPI

        :param data: list of bytes to send via SPI
        :return: data received after sending
        """

        tx_buf = (ctypes.c_uint8 * len(data))(*data)
        rx_buf = (ctypes.c_uint8 * len(data))()

        error_check(RA02_DYNLIB.spi_transcieve(
            ctypes.byref(self.spi),
            tx_buf,
            rx_buf,
            ctypes.c_size_t(len(data))
        ))

        return list(rx_buf)


class timeout_t(ctypes.Structure):
    """
    Defines timeout_t from timeout.h
    """
    _fields_ = [
        ('start', ctypes.c_uint64),
        ('duration', ctypes.c_uint64),
    ]

class Timeout:
    """
    Encapsulates timeout_t and timeout_* APIs from timeout.h
    """

    def __init__(self, duration: int = 0):
        """
        Initializes and starts timeout

        :param duration: Durations in milliseconds for timeout to expire. Can be 0
        """

        self.timeout = timeout_t()

        if duration:
            self.start(duration)

    def start(self, duration: int):
        """
        Starts timeout

        :param duration: Durations in milliseconds for timeout to expire. Can be 0
        """

        RA02_DYNLIB.timeout_start(ctypes.byref(self.timeout), duration)

    def restart(self):
        """
        Restarts timeout with same ticks values
        """

        RA02_DYNLIB.timeout_restart(ctypes.byref(self.timeout))

    def is_expired(self) -> bool:
        """
        Checks if timeout is expired

        :return: true if expired, false otherwise
        """

        return RA02_DYNLIB.timeout_is_expired(ctypes.byref(self.timeout))

    def expire(self):
        """
        Force expire a timeout
        """

        RA02_DYNLIB.timeout_expire(ctypes.byref(self.timeout))


class sx1278_cfg_t(ctypes.Structure):
    """
    Defines RA02 config from sx1278.h
    """
    _fields_ = [
        ('spi', ctypes.POINTER(spi_t)),
    ]

class sx1278_t(ctypes.Structure):
    """
    Defines RA02 context from sx1278.h
    """
    _fields_ = [
        ('spi', ctypes.POINTER(spi_t)),
        ('irq_flags', ctypes.c_uint8),
    ]

class Sx1278:
    """
    Encapsulates sx1278_t and sx1278_* APIs from sx1278.h
    """

    # Max packet size in bytes
    MAX_PAYLOAD = 64

    def __init__(self, spi: Spi = None):
        """
        Initializes RA02 driver

        :param spi: Initializes SPI handle. Can be None, but won't be initialized
        """

        self.sx1278 = sx1278_t()

        if spi:
            self.init(spi)

    def __del__(self):
        """
        Deinitializes RA02 driver
        """

        self.deinit()

    def init(self, spi: Spi):
        """
        Initializes RA02 driver

        :param spi: Initializes SPI handle. Can be None, but won't be initialized
        """

        self.spi = spi

        cfg = sx1278_cfg_t(spi=ctypes.pointer(spi.spi))

        error_check(RA02_DYNLIB.sx1278_init(ctypes.byref(self.sx1278), ctypes.byref(cfg)))

    def deinit(self):
        """
        Deinitializes RA02 driver
        """

        error_check(RA02_DYNLIB.sx1278_deinit(ctypes.byref(self.sx1278)))

    def reset(self):
        """
        Resets RA02
        """

        error_check(RA02_DYNLIB.sx1278_reset(ctypes.byref(self.sx1278)))

    def sleep(self):
        """
        Transitions RA02 to sleep mode
        """

        error_check(RA02_DYNLIB.sx1278_sleep(ctypes.byref(self.sx1278)))

    def set_freq(self, freq: int):
        """
        Sets frequency

        :param freq: Operating frequency in kHz
        """

        error_check(RA02_DYNLIB.sx1278_set_freq(ctypes.byref(self.sx1278), ctypes.c_uint32(freq)))

    def get_power(self):
        """
        Gets output power

        :return: current output power in db
        """

        db = ctypes.c_uint8()
        error_check(RA02_DYNLIB.sx1278_get_power(ctypes.byref(self.sx1278), ctypes.byref(db)))
        return db.value

    def set_power(self, db: int):
        """
        Sets output power

        :param db: output power in db
        """

        error_check(RA02_DYNLIB.sx1278_set_power(ctypes.byref(self.sx1278), ctypes.c_uint32(db)))

    def set_sync_word(self, sync_word: int):
        """
        Set sync word

        :param sync_word: Sync word
        """

        error_check(RA02_DYNLIB.sx1278_set_sync_word(ctypes.byref(self.sx1278), ctypes.c_uint32(sync_word)))

    def set_baudrate(self, baudrate: int):
        """
        Set baudrate

        :param baudrate: Baudrate
        """

        error_check(RA02_DYNLIB.sx1278_set_baudrate(ctypes.byref(self.sx1278), ctypes.c_uint32(baudrate)))

    def set_bandwidth(self, bandwidth: int):
        """
        Sets bandwidth

        :param bandwidth: Bandwidth
        """

        error_check(RA02_DYNLIB.sx1278_set_bandwidth(ctypes.byref(self.sx1278), ctypes.c_uint32(bandwidth)))

    def set_preamble(self, preamble: int):
        """
        Set preamble length

        :param preamble: Preamble length in bytes
        """
        error_check(RA02_DYNLIB.sx1278_set_preamble(ctypes.byref(self.sx1278), ctypes.c_uint32(preamble)))

    def set_sf(self, sf: int):
        """
        Set Spreading Factor

        :param sf: Spreading Factor
        """
        error_check(RA02_DYNLIB.sx1278_set_sf(ctypes.byref(self.sx1278), ctypes.c_uint8(sf)))

    def get_rssi(self):
        """
        Returns current measured RSSI

        :return: Current RSSI
        """

        rssi = ctypes.c_uint8()
        error_check(RA02_DYNLIB.sx1278_get_rssi(ctypes.byref(self.sx1278), ctypes.byref(rssi)))
        return rssi.value

    def poll_irq_flags(self):
        """
        Polls IRQ Flags register
        Reads value of RA02_LORA_REG_IRQ_FLAGS
        """

        RA02_DYNLIB.sx1278_poll_irq_flags(ctypes.byref(self.sx1278))

    def send(self, data: bytes):
        """
        Send data over radio

        :param data: list of bytes to send via radio
        """

        buf = (ctypes.c_uint8 * len(data))(*data)

        error_check(RA02_DYNLIB.sx1278_send(ctypes.byref(self.sx1278), buf, ctypes.c_size_t(len(data))))

    def recv(self, timeout: Timeout) -> bytes:
        """
        Receive data over radio for specified amount of time

        :param timeout: Initialized Timeout
        :return: list of received bytes, if receive was successful
        """

        buf = (ctypes.c_uint8 * self.MAX_PAYLOAD)()
        size = ctypes.c_size_t(self.MAX_PAYLOAD)

        error_check(RA02_DYNLIB.sx1278_recv(ctypes.byref(self.sx1278), buf, ctypes.byref(size), ctypes.byref(timeout.timeout)))

        return bytes(buf[:size.value])


def set_log_level(level: str):
    RA02_DYNLIB.log_set_level_from_str(level.encode('utf-8'))


def __init__(dynlib_path: str):
    """
    Initializes library. Loads RA02 dynamic library

    :param dynlib_path: Path to compiled shared object file
    """

    # Load dynamic library
    RA02_DYNLIB.load(dynlib_path)

    # spi.h

    # error_t spi_cfg_default(spi_cfg_t * cfg);
    RA02_DYNLIB.spi_cfg_default.argtypes = [ctypes.POINTER(spi_cfg_t)]
    RA02_DYNLIB.spi_cfg_default.restype = ctypes.c_int

    # error_t spi_init(spi_t * spi, spi_cfg_t * cfg, const char * dev);
    RA02_DYNLIB.spi_init.argtypes = [
        ctypes.POINTER(spi_t),
        ctypes.POINTER(spi_cfg_t),
        ctypes.c_char_p
    ]
    RA02_DYNLIB.spi_init.restype = ctypes.c_int

    # error_t spi_deinit(spi_t * spi);
    RA02_DYNLIB.spi_deinit.argtypes = [ctypes.POINTER(spi_t)]
    RA02_DYNLIB.spi_deinit.restype = ctypes.c_int

    # error_t spi_transcieve(spi_t * spi, uint8_t * tx_buf, uint8_t * rx_buf, size_t size);
    RA02_DYNLIB.spi_transcieve.argtypes = [
        ctypes.POINTER(spi_t),
        ctypes.POINTER(ctypes.c_uint8),
        ctypes.POINTER(ctypes.c_uint8),
        ctypes.c_size_t
    ]
    RA02_DYNLIB.spi_transcieve.restype = ctypes.c_int

    # timeout.h

    # void timeout_start(timeout_t * timeout, uint64_t ms);
    RA02_DYNLIB.timeout_start.argtypes = [ctypes.POINTER(timeout_t), ctypes.c_uint64]
    RA02_DYNLIB.timeout_start.restype = None

    # void timeout_restart(timeout_t * timeout);
    RA02_DYNLIB.timeout_restart.argtypes = [ctypes.POINTER(timeout_t)]
    RA02_DYNLIB.timeout_restart.restype = None

    # bool timeout_is_expired(const timeout_t * timeout);
    RA02_DYNLIB.timeout_is_expired.argtypes = [ctypes.POINTER(timeout_t)]
    RA02_DYNLIB.timeout_is_expired.restype = ctypes.c_bool

    # void timeout_expire(timeout_t * timeout);
    RA02_DYNLIB.timeout_expire.argtypes = [ctypes.POINTER(timeout_t)]
    RA02_DYNLIB.timeout_expire.restype = None

    # sx1278.h

    # error_t sx1278_init(sx1278_t * sx1278, sx1278_cfg_t * cfg);
    RA02_DYNLIB.sx1278_init.argtypes = [ctypes.POINTER(sx1278_t), ctypes.POINTER(sx1278_cfg_t)]
    RA02_DYNLIB.sx1278_init.restype = ctypes.c_int

    # error_t sx1278_deinit(sx1278_t * sx1278);
    RA02_DYNLIB.sx1278_deinit.argtypes = [ctypes.POINTER(sx1278_t)]
    RA02_DYNLIB.sx1278_deinit.restype = ctypes.c_int

    # error_t sx1278_reset(sx1278_t * sx1278);
    RA02_DYNLIB.sx1278_reset.argtypes = [ctypes.POINTER(sx1278_t)]
    RA02_DYNLIB.sx1278_reset.restype = ctypes.c_int

    # error_t sx1278_sleep(sx1278_t * sx1278);
    RA02_DYNLIB.sx1278_sleep.argtypes = [ctypes.POINTER(sx1278_t)]
    RA02_DYNLIB.sx1278_sleep.restype = ctypes.c_int

    # error_t sx1278_set_freq(sx1278_t * sx1278, uint32_t khz);
    RA02_DYNLIB.sx1278_set_freq.argtypes = [ctypes.POINTER(sx1278_t), ctypes.c_uint32]
    RA02_DYNLIB.sx1278_set_freq.restype = ctypes.c_int

    # error_t sx1278_get_power(sx1278_t * sx1278, uint8_t * db);
    RA02_DYNLIB.sx1278_get_power.argtypes = [ctypes.POINTER(sx1278_t), ctypes.POINTER(ctypes.c_uint8)]
    RA02_DYNLIB.sx1278_get_power.restype = ctypes.c_int

    # error_t sx1278_set_power(sx1278_t * sx1278, uint8_t db);
    RA02_DYNLIB.sx1278_set_power.argtypes = [ctypes.POINTER(sx1278_t), ctypes.c_uint32]
    RA02_DYNLIB.sx1278_set_power.restype = ctypes.c_int

    # error_t sx1278_set_sync_word(sx1278_t * sx1278, uint32_t sync_word);
    RA02_DYNLIB.sx1278_set_sync_word.argtypes = [ctypes.POINTER(sx1278_t), ctypes.c_uint32]
    RA02_DYNLIB.sx1278_set_sync_word.restype = ctypes.c_int

    # error_t sx1278_set_baudrate(sx1278_t * sx1278, uint32_t baudrate);
    RA02_DYNLIB.sx1278_set_baudrate.argtypes = [ctypes.POINTER(sx1278_t), ctypes.c_uint32]
    RA02_DYNLIB.sx1278_set_baudrate.restype = ctypes.c_int

    # error_t sx1278_set_bandwidth(sx1278_t * sx1278, uint32_t bandwidth);
    RA02_DYNLIB.sx1278_set_bandwidth.argtypes = [ctypes.POINTER(sx1278_t), ctypes.c_uint32]
    RA02_DYNLIB.sx1278_set_bandwidth.restype = ctypes.c_int

    # error_t sx1278_set_preamble(sx1278_t * sx1278, uint32_t preamble);
    RA02_DYNLIB.sx1278_set_preamble.argtypes = [ctypes.POINTER(sx1278_t), ctypes.c_uint32]
    RA02_DYNLIB.sx1278_set_preamble.restype = ctypes.c_int

    # error_t sx1278_set_sf(sx1278_t * sx1278, uint8_t sf);
    RA02_DYNLIB.sx1278_set_sf.argtypes = [ctypes.POINTER(sx1278_t), ctypes.c_uint8]
    RA02_DYNLIB.sx1278_set_sf.restype = ctypes.c_int

    # error_t sx1278_get_rssi(sx1278_t * sx1278, int8_t * rssi);
    RA02_DYNLIB.sx1278_get_rssi.argtypes = [ctypes.POINTER(sx1278_t), ctypes.POINTER(ctypes.c_uint8)]
    RA02_DYNLIB.sx1278_get_rssi.restype = ctypes.c_int

    # error_t sx1278_poll_irq_flags(sx1278_t * sx1278);
    RA02_DYNLIB.sx1278_poll_irq_flags.argtypes = [ctypes.POINTER(sx1278_t)]
    RA02_DYNLIB.sx1278_poll_irq_flags.restype = ctypes.c_int

    # error_t sx1278_send(sx1278_t * sx1278, uint8_t * buf, size_t size);
    RA02_DYNLIB.sx1278_send.argtypes = [
        ctypes.POINTER(sx1278_t),
        ctypes.POINTER(ctypes.c_uint8),
        ctypes.c_size_t
    ]
    RA02_DYNLIB.sx1278_send.restype = ctypes.c_int

    # error_t sx1278_recv(sx1278_t * sx1278, uint8_t * buf, size_t * size, timeout_t * timeout);
    RA02_DYNLIB.sx1278_recv.argtypes = [
        ctypes.POINTER(sx1278_t),
        ctypes.POINTER(ctypes.c_uint8),
        ctypes.POINTER(ctypes.c_size_t),
        ctypes.POINTER(timeout_t)
    ]
    RA02_DYNLIB.sx1278_recv.restype = ctypes.c_int

    # void log_set_level(log_level_t level);
    RA02_DYNLIB.log_set_level_from_str.argtypes = [
        ctypes.c_char_p
    ]
    RA02_DYNLIB.sx1278_recv.restype = None
