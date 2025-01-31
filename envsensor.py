from machine import SoftI2C, Pin
import time
import struct


class EnvSensor:
    # Constants
    SCD40_ADDR = 0x62
    SHT30_ADDR = 0x44

    # SCD40 Commands
    SCD40_START_MEASURE = 0x21B1
    SCD40_READ_MEASURE = 0xEC05
    SCD40_STOP_MEASURE = 0x3F86

    # SHT30 Commands
    SHT30_MEASURE_HIGH = 0x2C06

    def __init__(self, i2c, sensor_type):
        self.i2c = i2c
        if sensor_type not in ['SCD40', 'SHT30']:
            raise ValueError("Invalid sensor type. Choose 'SCD40' or 'SHT30'.")
        self.sensor_type = sensor_type
        self.addr = self.SCD40_ADDR if sensor_type == 'SCD40' else self.SHT30_ADDR

    def start_measurement(self):
        if self.sensor_type == 'SCD40':
            self._write_command(self.SCD40_START_MEASURE)
            time.sleep(5)  # Wait for SCD40 to initialize

    def stop_measurement(self):
        if self.sensor_type == 'SCD40':
            self._write_command(self.SCD40_STOP_MEASURE)

    def read_measurement(self):
        if self.sensor_type == 'SCD40':
            return self._read_scd40()
        else:
            return self._read_sht30()

    def _read_scd40(self):
        self._write_command(self.SCD40_READ_MEASURE)
        time.sleep_ms(1)

        data = self.i2c.readfrom(self.addr, 9)
        co2 = struct.unpack(">H", data[0:2])[0]
        co2_crc = data[2]
        temp = struct.unpack(">H", data[3:5])[0]
        temp_crc = data[5]
        hum = struct.unpack(">H", data[6:8])[0]
        hum_crc = data[8]

        if (self._crc8(co2) != co2_crc or
                self._crc8(temp) != temp_crc or
                self._crc8(hum) != hum_crc):
            raise ValueError("CRC check failed")

        co2 = co2
        temperature = -45 + 175 * (temp / 65535)
        humidity = 100 * (hum / 65535)
        return co2, temperature, humidity

    def _read_sht30(self):
        self._write_command(self.SHT30_MEASURE_HIGH)
        time.sleep_ms(50)
        data = self.i2c.readfrom(self.addr, 6)

        temp = struct.unpack(">H", data[0:2])[0]
        temp_crc = data[2]
        hum = struct.unpack(">H", data[3:5])[0]
        hum_crc = data[5]

        if self._crc8(temp) != temp_crc or self._crc8(hum) != hum_crc:
            raise ValueError("CRC check failed")

        temperature = -45 + 175 * (temp / 65535)
        humidity = 100 * (hum / 65535)
        return None, temperature, humidity  # None for CO2 as SHT30 doesn't measure it

    def _write_command(self, cmd):
        self.i2c.writeto(self.addr, struct.pack(">H", cmd))

    def _crc8(self, data):
        crc = 0xFF
        for i in range(2):
            crc ^= (data >> (8 - i * 8)) & 0xFF
            for _ in range(8):
                if crc & 0x80:
                    crc = (crc << 1) ^ 0x31
                else:
                    crc <<= 1
                crc &= 0xFF
        return crc