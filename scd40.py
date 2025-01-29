from machine import I2C
import time
import struct

class SCD40:
    # SCD40 I2C address
    SCD40_I2C_ADDR = 0x62

    # SCD40 commands
    CMD_START_PERIODIC_MEASUREMENT = 0x21B1
    CMD_READ_MEASUREMENT = 0xEC05
    CMD_STOP_PERIODIC_MEASUREMENT = 0x3F86
    CMD_SET_TEMPERATURE_OFFSET = 0x241D
    CMD_GET_TEMPERATURE_OFFSET = 0x2318
    CMD_SET_ALTITUDE = 0x2427
    CMD_GET_ALTITUDE = 0x2322
    CMD_SET_AMBIENT_PRESSURE = 0xE000
    CMD_PERFORM_FORCED_RECALIBRATION = 0x362F
    CMD_SET_AUTOMATIC_SELF_CALIBRATION = 0x2416
    CMD_GET_AUTOMATIC_SELF_CALIBRATION = 0x2313

    def __init__(self, i2c):
        self.i2c = i2c

    def _write_command(self, command):
        self.i2c.writeto(self.SCD40_I2C_ADDR, struct.pack(">H", command))

    def _read_data(self, size=3):
        return self.i2c.readfrom(self.SCD40_I2C_ADDR, size)

    def start_periodic_measurement(self):
        self._write_command(self.CMD_START_PERIODIC_MEASUREMENT)
        time.sleep_ms(1)

    def stop_periodic_measurement(self):
        self._write_command(self.CMD_STOP_PERIODIC_MEASUREMENT)
        time.sleep_ms(500)

    def read_measurement(self):
        self._write_command(self.CMD_READ_MEASUREMENT)
        time.sleep_ms(1)
        data = self._read_data(9)

        co2, co2_crc = struct.unpack(">HB", data[0:3])
        temp, temp_crc = struct.unpack(">HB", data[3:6])
        hum, hum_crc = struct.unpack(">HB", data[6:9])

        if (self._crc8(co2) != co2_crc or
                self._crc8(temp) != temp_crc or
                self._crc8(hum) != hum_crc):
            raise ValueError("CRC check failed")

        co2 = co2
        temperature = -45 + 175 * (temp / 65535)
        humidity = 100 * (hum / 65535)

        return co2, temperature, humidity

    def set_temperature_offset(self, offset):
        offset = int(offset * 100)
        self._write_command(self.CMD_SET_TEMPERATURE_OFFSET)
        self.i2c.writeto(self.SCD40_I2C_ADDR, struct.pack(">H", offset))
        time.sleep_ms(1)

    def get_temperature_offset(self):
        self._write_command(self.CMD_GET_TEMPERATURE_OFFSET)
        time.sleep_ms(1)
        data = self._read_data(3)
        offset = struct.unpack(">H", data[0:2])[0]
        return offset / 100.0

    def perform_forced_recalibration(self, target_co2):
        self._write_command(self.CMD_PERFORM_FORCED_RECALIBRATION)
        self.i2c.writeto(self.SCD40_I2C_ADDR, struct.pack(">H", target_co2))
        time.sleep_ms(400)
        data = self._read_data(3)
        frc_correction = struct.unpack(">H", data[0:2])[0]
        return frc_correction

    def set_automatic_self_calibration(self, enable):
        self._write_command(self.CMD_SET_AUTOMATIC_SELF_CALIBRATION)
        self.i2c.writeto(self.SCD40_I2C_ADDR, struct.pack(">H", 1 if enable else 0))
        time.sleep_ms(1)

    def get_automatic_self_calibration(self):
        self._write_command(self.CMD_GET_AUTOMATIC_SELF_CALIBRATION)
        time.sleep_ms(1)
        data = self._read_data(3)
        return bool(struct.unpack(">H", data[0:2])[0])

    def _crc8(self, data):
        crc = 0xFF
        for byte in data.to_bytes(2, 'big'):
            crc ^= byte
            for _ in range(8):
                if crc & 0x80:
                    crc = (crc << 1) ^ 0x31
                else:
                    crc <<= 1
            crc &= 0xFF
        return crc