"""TI INA238 power-monitor driver.

Cloned from ARK-OS `platform/jetson/scripts/ina238_test.py` and hardened for
unattended use: the die-temperature decode is fixed for sub-zero readings, the
bus handle is a context manager, and reads raise a typed error the caller can
distinguish from a bad measurement.

Datasheet: TI INA238, SLUSCY9.
"""

from __future__ import annotations

import math

from smbus2 import SMBus

# --- Register map ----------------------------------------------------------
REG_CONFIG = 0x00
REG_ADC_CONFIG = 0x01
REG_SHUNT_CAL = 0x02
REG_VSHUNT = 0x04
REG_VBUS = 0x05
REG_DIETEMP = 0x06
REG_CURRENT = 0x07
REG_POWER = 0x08
REG_MANUFACTURER_ID = 0x3E
REG_DEVICE_ID = 0x3F

MANUFACTURER_ID_TI = 0x5449  # "TI"

# --- Conversion factors (ADCRANGE = 0) -------------------------------------
VSHUNT_LSB = 5e-6  # 5 uV/LSB
VBUS_LSB = 3.125e-3  # 3.125 mV/LSB
TEMP_LSB = 0.125  # 125 m degC/LSB
POWER_LSB_MULTIPLIER = 0.2


class INA238Error(RuntimeError):
    """An I2C transaction with the INA238 failed."""


class INA238:
    """Single INA238 on an I2C bus.

    `i_max` sets the current resolution. For a shunt rated at `power_rating`
    watts the thermally-safe maximum is sqrt(power_rating / r_shunt); see
    `from_shunt_rating`.
    """

    def __init__(self, bus_num: int = 7, address: int = 0x45,
                 r_shunt: float = 0.001, i_max: float = 70.7) -> None:
        self.bus_num = bus_num
        self.address = address
        self.r_shunt = r_shunt

        # Current_LSB = MaxExpectedCurrent / 2^15
        self.current_lsb = i_max / 32768
        self.power_lsb = self.current_lsb * POWER_LSB_MULTIPLIER
        # SHUNT_CAL = 819.2e6 x CURRENT_LSB x R_shunt
        self.shunt_cal = int(819.2e6 * self.current_lsb * self.r_shunt)

        try:
            self.bus = SMBus(bus_num)
        except OSError as exc:
            raise INA238Error(f"cannot open I2C bus {bus_num}: {exc}") from exc

        self.configure()

    @classmethod
    def from_shunt_rating(cls, bus_num: int = 7, address: int = 0x45,
                          r_shunt: float = 0.001,
                          power_rating: float = 5.0) -> "INA238":
        """Build with `i_max` derived from the shunt's power rating.

        power = I^2 * R, so I_max = sqrt(P / R). For the stock 1 mOhm 5 W
        shunt that is 70.71 A.
        """
        return cls(bus_num=bus_num, address=address, r_shunt=r_shunt,
                   i_max=math.sqrt(power_rating / r_shunt))

    # --- Setup -------------------------------------------------------------
    def configure(self) -> None:
        # CONFIG: ADCRANGE = 0 (+/-163.84 mV), CONVDLY = 0.
        self.write_register(REG_CONFIG, 0x0000)

        # ADC_CONFIG: continuous on bus + shunt + temperature, 1052 us
        # conversion time on each, 16-sample hardware averaging.
        adc_config = (0xF << 12) | (5 << 9) | (5 << 6) | (5 << 3) | 2
        self.write_register(REG_ADC_CONFIG, adc_config)

        # SHUNT_CAL encodes the shunt resistance and sets the resolution of
        # the CURRENT register.
        self.write_register(REG_SHUNT_CAL, self.shunt_cal)

    def probe(self) -> bool:
        """True if the device at this address identifies as a TI part.

        Guards against a config that points at an address where something
        else answers -- that would otherwise read as a plausible voltage.
        """
        try:
            return self.read_register(REG_MANUFACTURER_ID, signed=False) == MANUFACTURER_ID_TI
        except INA238Error:
            return False

    # --- Raw I/O -----------------------------------------------------------
    def write_register(self, reg: int, value: int) -> None:
        data = [(value >> 8) & 0xFF, value & 0xFF]
        try:
            self.bus.write_i2c_block_data(self.address, reg, data)
        except OSError as exc:
            raise INA238Error(f"write 0x{reg:02X} failed: {exc}") from exc

    def read_register(self, reg: int, length: int = 2, signed: bool = True) -> int:
        try:
            data = self.bus.read_i2c_block_data(self.address, reg, length)
        except OSError as exc:
            raise INA238Error(f"read 0x{reg:02X} failed: {exc}") from exc
        return int.from_bytes(data, byteorder="big", signed=signed)

    # --- Measurements ------------------------------------------------------
    def read_shunt_voltage(self) -> float:
        return self.read_register(REG_VSHUNT) * VSHUNT_LSB

    def read_bus_voltage(self) -> float:
        # VBUS is unsigned 16-bit; bit 15 is reserved and reads 0.
        return self.read_register(REG_VBUS, signed=False) * VBUS_LSB

    def read_temperature(self) -> float:
        # DIETEMP holds a 12-bit two's-complement value in the top bits. Read
        # the word unsigned first: shifting a Python int that is already
        # negative sign-extends, and the 0x800 sign test below then never
        # fires, so sub-zero die temperatures decode as large positives.
        raw = self.read_register(REG_DIETEMP, signed=False)
        raw_temp = raw >> 4
        if raw_temp & 0x800:
            raw_temp -= 1 << 12
        return raw_temp * TEMP_LSB

    def read_current(self) -> float:
        # Current [A] = CURRENT_LSB x CURRENT
        return self.read_register(REG_CURRENT) * self.current_lsb

    def read_power(self) -> float:
        # Power [W] = 0.2 x CURRENT_LSB x POWER (24-bit unsigned)
        return self.read_register(REG_POWER, length=3, signed=False) * self.power_lsb

    # --- Lifecycle ---------------------------------------------------------
    def close(self) -> None:
        try:
            self.bus.close()
        except OSError:
            pass

    def __enter__(self) -> "INA238":
        return self

    def __exit__(self, *_exc) -> None:
        self.close()
