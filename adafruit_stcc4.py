# SPDX-FileCopyrightText: Copyright (c) 2026 Liz Clark for Adafruit Industries
#
# SPDX-License-Identifier: MIT
"""
`adafruit_stcc4`
================================================================================

CircuitPython driver for the STCC4 and SHT41 - CO2, Temperature and Humidity Sensor


* Author(s): Liz Clark

Implementation Notes
--------------------

**Hardware:**

* `Adafruit STCC4 and SHT41 - CO2, Temperature & Humidity Sensor <https://www.adafruit.com/product/6478>`_

**Software and Dependencies:**

* Adafruit CircuitPython firmware for the supported boards:
  https://circuitpython.org/downloads

* Adafruit's Bus Device library: https://github.com/adafruit/Adafruit_CircuitPython_BusDevice

"""

import struct
import time

from adafruit_bus_device.i2c_device import I2CDevice
from micropython import const

try:
    from typing import Tuple

    from busio import I2C
except ImportError:
    pass

__version__ = "0.0.0+auto.0"
__repo__ = "https://github.com/adafruit/Adafruit_CircuitPython_STCC4.git"

_STCC4_DEFAULT_ADDR = const(0x64)
_STCC4_PRODUCT_ID = 0x0901018A

_START_CONTINUOUS = const(0x218B)
_STOP_CONTINUOUS = const(0x3F86)
_READ_MEASUREMENT = const(0xEC05)
_SET_RHT_COMPENSATION = const(0xE000)
_SET_PRESSURE_COMPENSATION = const(0xE016)
_MEASURE_SINGLE_SHOT = const(0x219D)
_ENTER_SLEEP = const(0x3650)
_EXIT_SLEEP = const(0x00)
_PERFORM_CONDITIONING = const(0x29BC)
_SOFT_RESET = const(0x06)
_FACTORY_RESET = const(0x3632)
_SELF_TEST = const(0x278C)
_ENABLE_TESTING = const(0x3FBC)
_DISABLE_TESTING = const(0x3F3D)
_FORCED_RECALIBRATION = const(0x362F)
_GET_PRODUCT_ID = const(0x365B)

# Status bit masks
STATUS_VOLTAGE_ERROR = const(0x0001)
STATUS_DEBUG_MASK = const(0x000E)
STATUS_SHT_NOT_CONNECTED = const(0x0010)
STATUS_MEMORY_ERROR_MASK = const(0x0060)
STATUS_TESTING_MODE = const(0x4000)


class STCC4:
    def __init__(self, i2c_bus: I2C, address: int = _STCC4_DEFAULT_ADDR) -> None:
        self.i2c_device = I2CDevice(i2c_bus, address)

        self.reset()
        pid = self.product_id
        if pid != _STCC4_PRODUCT_ID:
            raise RuntimeError(
                f"Failed to find STCC4 - expected product ID {_STCC4_PRODUCT_ID}, " f"got 0x{pid}"
            )

        self._co2: int = 0
        self._temperature: float = 0.0
        self._humidity: float = 0.0
        self._status: int = 0
        self._continuous: bool = False

    @staticmethod
    def _crc8(data: bytes) -> int:
        crc = 0xFF
        for byte in data:
            crc ^= byte
            for _ in range(8):
                if crc & 0x80:
                    crc = (crc << 1) ^ 0x31
                else:
                    crc = crc << 1
                crc &= 0xFF
        return crc

    def _write_command(self, command: int) -> None:
        buf = struct.pack(">H", command)
        with self.i2c_device as i2c:
            i2c.write(buf)

    def _read_command(self, command: int, word_count: int) -> bytearray:
        cmd_buf = struct.pack(">H", command)
        read_len = word_count * 3
        reply = bytearray(read_len)
        with self.i2c_device as i2c:
            i2c.write_then_readinto(cmd_buf, reply)

        for i in range(word_count):
            offset = i * 3
            if self._crc8(reply[offset : offset + 2]) != reply[offset + 2]:
                raise RuntimeError(
                    f"CRC mismatch at word {i} in response to command 0x{command:04X}"
                )
        return reply

    def _read_words(self, command: int, word_count: int) -> Tuple[int, ...]:
        raw = self._read_command(command, word_count)
        words = []
        for i in range(word_count):
            offset = i * 3
            words.append((raw[offset] << 8) | raw[offset + 1])
        return tuple(words)

    def _write_command_with_arg(self, command: int, arg: int) -> None:
        arg_bytes = struct.pack(">H", arg)
        crc = self._crc8(arg_bytes)
        buf = struct.pack(">H", command) + arg_bytes + bytes([crc])
        with self.i2c_device as i2c:
            i2c.write(buf)

    def _read_measurement(self) -> None:
        words = self._read_words(_READ_MEASUREMENT, 4)
        self._co2 = words[0]
        self._temperature = words[1] * 175.0 / 65536.0 - 45.0
        self._humidity = words[2] * 125.0 / 65536.0 - 6.0
        self._status = words[3]

    @property
    def CO2(self) -> int:
        if not self._continuous:
            self.measure_single_shot()
        self._read_measurement()
        return self._co2

    @property
    def temperature(self) -> float:
        return self._temperature

    @property
    def relative_humidity(self) -> float:
        return self._humidity

    @property
    def status(self) -> int:
        return self._status

    def measure_single_shot(self) -> None:
        self._write_command(_MEASURE_SINGLE_SHOT)
        time.sleep(0.5)  # Single shot measurement time

    @property
    def continuous_measurement(self) -> bool:
        return self._continuous

    @continuous_measurement.setter
    def continuous_measurement(self, value: bool) -> None:
        if value:
            self._write_command(_START_CONTINUOUS)
        else:
            self._write_command(_STOP_CONTINUOUS)
        self._continuous = value
        if value:
            time.sleep(1)  # Wait for first measurement

    def set_pressure_compensation(self, pressure_hpa: int) -> None:
        self._write_command_with_arg(_SET_PRESSURE_COMPENSATION, pressure_hpa)

    def set_rht_compensation(self, rht_value: int) -> None:
        self._write_command_with_arg(_SET_RHT_COMPENSATION, rht_value)

    def perform_conditioning(self) -> None:
        self._write_command(_PERFORM_CONDITIONING)
        time.sleep(22)

    def forced_recalibration(self, reference_co2: int) -> int:
        self._write_command_with_arg(_FORCED_RECALIBRATION, reference_co2)
        time.sleep(0.5)
        words = self._read_words(_FORCED_RECALIBRATION, 1)
        return words[0]

    @property
    def product_id(self) -> int:
        words = self._read_words(_GET_PRODUCT_ID, 2)
        return (words[0] << 16) | words[1]

    def reset(self) -> None:
        buf = bytes([_SOFT_RESET])
        with self.i2c_device as i2c:
            i2c.write(buf)
        time.sleep(0.01)

    def factory_reset(self) -> None:
        self._write_command(_FACTORY_RESET)
        time.sleep(0.1)

    def self_test(self) -> int:
        self._write_command(_SELF_TEST)
        time.sleep(0.36)
        words = self._read_words(_SELF_TEST, 1)
        return words[0]

    @property
    def sleep_mode(self) -> None:
        raise AttributeError("sleep_mode is write-only")

    @sleep_mode.setter
    def sleep_mode(self, enable: bool) -> None:
        if enable:
            self._write_command(_ENTER_SLEEP)
            time.sleep(0.001)
        else:
            buf = bytes([_EXIT_SLEEP])
            with self.i2c_device as i2c:
                i2c.write(buf)
            time.sleep(0.005)
