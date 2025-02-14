import gc
import os
import time
import struct
import machine
from machine import I2C, Pin, PWM, Timer, RTC
import _thread
from collections import deque
from umqtt.simple import MQTTClient
import network
import ntptime
import utime
import json
import urequests

# Sensor class definition
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

class BoundedDeque:
    def __init__(self, maxlen):
        self.maxlen = maxlen
        self.queue = deque((), maxlen)

    def append(self, item):
        if len(self.queue) >= self.maxlen:
            self.queue.popleft()
        self.queue.append(item)

    def popleft(self):
        return self.queue.popleft()

    def __len__(self):
        return len(self.queue)

# Pin mapping for M5Stack Station
LED_PIN             = 17
VENT_PIN            = 25
WATER_PIN           = 26
MEASUREMENT_RATE_S  = 30
PWM_FREQUENCY       = 100

# Cycle settings
VENT_WORK_TIME_MIN    = 3
WATER_WORK_TIME_MIN   = 1

DEFAULT_DAY_START_HR  = 6
DEFAULT_DAY_START_MIN = 0
DEFAULT_DAY_DUR       = 18
DEFAULT_TIMEZONE      = 7
DEFAULT_LIGHT_PWM     = 20
DEFAULT_WTR_MAX_CNTR  = 2
DEFAULT_VENT_MAX_CNTR = 72
DEFAULT_LIGHT_AUTO    = 1
DEFAULT_WATER_AUTO    = 1
DEFAULT_VENT_AUTO     = 1
VENT_PWM_VALUE        = 80


# Wi-Fi settings
SSID        = "ELTEX-8A08"
PASSWORD    = "GP21204758"

# OTA Server Settings
SERVER_URL = "http://178.236.244.174:5000"
CURRENT_VERSION = "5.0.0"

# MQTT settings
MQTT_HOST       = "dev.rightech.io"
MQTT_PORT       = 1883
MQTT_CLIENT_ID  = "gbu_dev_3"

# Sensors topics (publish)
MQTT_TOPIC_TEMP     = MQTT_CLIENT_ID + "/Scd40/t"
MQTT_TOPIC_HUM      = MQTT_CLIENT_ID + "/Scd40/rh"
MQTT_TOPIC_CO2      = MQTT_CLIENT_ID + "/Scd40/cd"
MQTT_TOPIC_TEMP_EXT = MQTT_CLIENT_ID + "/Sht30/t_ext"
MQTT_TOPIC_HUM_EXT  = MQTT_CLIENT_ID + "/Sht30/h_ext"
MQTT_TOPIC_LUX      = MQTT_CLIENT_ID + "/Bh1750/lux"
MQTT_TOPIC_PWM      = MQTT_CLIENT_ID + "/relay/light_pwm"

# Settings topics (subscribe)
MQTT_TOPIC_SETTING_DAY_START_HR  = MQTT_CLIENT_ID + "/settings/day_start_hr"
MQTT_TOPIC_SETTING_DAY_START_MIN = MQTT_CLIENT_ID + "/settings/day_start_min"
MQTT_TOPIC_SETTING_DAY_DUR       = MQTT_CLIENT_ID + "/settings/day_dur"
MQTT_TOPIC_SETTING_TIMEZONE      = MQTT_CLIENT_ID + "/settings/tzn"
MQTT_TOPIC_SETTING_LIGHT_PWM     = MQTT_CLIENT_ID + "/settings/light_pwm"
MQTT_TOPIC_SETTING_WTR_MAX_CNTR  = MQTT_CLIENT_ID + "/settings/wtr_max_cnt"
MQTT_TOPIC_SETTING_VENT_MAX_CNTR = MQTT_CLIENT_ID + "/settings/vent_max_cnt"
MQTT_TOPIC_SETTING_LIGHT_AUTO    = MQTT_CLIENT_ID + "/settings/light_auto"
MQTT_TOPIC_SETTING_WATER_AUTO    = MQTT_CLIENT_ID + "/settings/water_auto"
MQTT_TOPIC_SETTING_VENT_AUTO     = MQTT_CLIENT_ID + "/settings/vent_auto"
# MQTT_TOPIC_SETTING_CO2_AUTO      = "gbu_dev_3/settings/co2_auto"
# MQTT_TOPIC_SETTING_WTR_DUR       = "gbu_dev_3/settings/wtr_dur"
# MQTT_TOPIC_SETTING_WTR_OFFSET    = "gbu_dev_3/settings/wtr_offset"
# MQTT_TOPIC_SETTING_GLOB_AUTO     = "gbu_dev_3/settings/global_auto"

# Commands topic (subscribe)
MQTT_TOPIC_WATER_CON        = MQTT_CLIENT_ID + "/relay/water"
MQTT_TOPIC_VENT_CON         = MQTT_CLIENT_ID + "/relay/vent"
MQTT_TOPIC_VENT_PWM         = MQTT_CLIENT_ID + "/relay/vent_pwm"
MQTT_TOPIC_LIGHT_CON        = MQTT_CLIENT_ID + "/relay/light"
MQTT_TOPIC_LIGHT_PWM        = MQTT_CLIENT_ID + "/relay/light_pwm"
MQTT_TOPIC_REQ_ALL_SETTINGS = MQTT_CLIENT_ID + "/cmd/req_all"
MQTT_TOPIC_REBOOT           = MQTT_CLIENT_ID + "/cmd/reboot"

# Status topics (publish)
MQTT_TOPIC_STATUS_DAY_START_HR  = MQTT_CLIENT_ID + "/status/day_start_hr"
MQTT_TOPIC_STATUS_DAY_START_MIN = MQTT_CLIENT_ID + "/status/day_start_min"
MQTT_TOPIC_STATUS_DAY_DUR       = MQTT_CLIENT_ID + "/status/day_dur"
MQTT_TOPIC_STATUS_TIMEZONE      = MQTT_CLIENT_ID + "/status/tzn"
MQTT_TOPIC_STATUS_LIGHT_PWM     = MQTT_CLIENT_ID + "/status/light_pwm"
MQTT_TOPIC_STATUS_WTR_MAX_CNTR  = MQTT_CLIENT_ID + "/status/wtr_max_cnt"
MQTT_TOPIC_STATUS_VENT_MAX_CNTR = MQTT_CLIENT_ID + "/status/vent_max_cnt"
MQTT_TOPIC_STATUS_LIGHT_AUTO    = MQTT_CLIENT_ID + "/status/light_auto"
MQTT_TOPIC_STATUS_WATER_AUTO    = MQTT_CLIENT_ID + "/status/water_auto"
MQTT_TOPIC_STATUS_VENT_AUTO     = MQTT_CLIENT_ID + "/status/vent_auto"

# System topics (publish)
MQTT_TOPIC_STATUS_IPADDR = MQTT_CLIENT_ID + "/status/ip_addr"
MQTT_TOPIC_STATUS_SWVER = MQTT_CLIENT_ID + "/status/sw_ver"

# Global variables and objects
data_lock = _thread.allocate_lock()

rtc = RTC()
JSON_FILE = 'config.json'
mqtt_queue = BoundedDeque(maxlen=20)
queue_lock = _thread.allocate_lock()
# vent_pwm_pin = PWM(Pin(VENT_PIN))
# light_pwm_pin = PWM(Pin(LED_PIN))
# water_pin = Pin(WATER_PIN)

def check_for_update():
    print("Проверка обновлений...")
    try:
        response = urequests.get(f"{SERVER_URL}/check_update?version={CURRENT_VERSION}")
        if response.status_code == 200:
            update_info = response.json()
            if update_info.get('update_available', False):
                print("Доступно обновление. Начинаем загрузку...")
                return download_update()
            else:
                print("Обновление не требуется")
        else:
            print("Ошибка при проверке обновлений. Код статуса:", response.status_code)
    except Exception as e:
        print("Ошибка при проверке обновлений:", str(e))
    return False

def download_update():
    try:
        gc.collect()
        response = urequests.get(f"{SERVER_URL}/update")
        if response.status_code == 200:
            update_files = response.json()
            for file_name, file_content in update_files.items():
                with open(file_name, "w") as f:
                    f.write(file_content)
                print(f"Файл {file_name} обновлен")
            return True
        else:
            print("Ошибка при загрузке обновлений. Код статуса:", response.status_code)
    except Exception as e:
        print("Ошибка при загрузке обновлений:", str(e))
    return False

def update_check_task():
    while True:
        if check_for_update():
            print("Обновление успешно загружено. Перезагрузка...")
            machine.reset()
        utime.sleep(60)

def check_schedule(equipment):
    current_time = rtc.datetime()
    offset_minutes = get_value('tzn') * 60
    current_minutes = (current_time[4] * 60 + current_time[5] + offset_minutes) % 1440
    start_minutes = get_value('day_start_hr') * 60 + get_value('day_start_min')
    end_minutes = start_minutes + get_value('day_dur') * 60
    print(f'start_minutes: {start_minutes}')
    print(f'end_minutes: {end_minutes}')
    print(f'current_minutes: {current_minutes}')

    if equipment == 'light':

        if start_minutes <= current_minutes < end_minutes:
            return True
        else:
            return False
    elif equipment == 'water':
        if start_minutes + 60 <= current_minutes < end_minutes - 60:
            return True
        else:
            return False

def file_exists(filename):
    try:
        os.stat(filename)
        return True
    except OSError:
        return False

def load_json():
    if file_exists(JSON_FILE):
        with open(JSON_FILE, 'r') as f:
            return json.load(f)
    return {}

def save_json(data):
    with open(JSON_FILE, 'w') as f:
        json.dump(data, f)

def update_value(key, value):
    data = load_json()
    data[key] = value
    save_json(data)
    print(f"Updated {key} to {value}")

def get_value(key, default=None):
    data = load_json()
    return data.get(key, default)

def connect_wifi(ssid, password, max_attempts=5, retry_delay=5):
    wlan = network.WLAN(network.STA_IF)
    wlan.active(True)

    if wlan.isconnected():
        print("Already connected to WiFi")
        return wlan.ifconfig()[0]

    print(f"Connecting to WiFi network: {ssid}")

    for attempt in range(max_attempts):
        wlan.disconnect()
        time.sleep(1)
        wlan.connect(ssid, password)

        # Wait for connection with timeout
        start_time = time.time()
        while not wlan.isconnected() and time.time() - start_time < 10:
            time.sleep(0.5)

        if wlan.isconnected():
            # print("WiFi connected. Network config:", wlan.ifconfig())
            print(f'WiFi connected, IP addr is {wlan.ifconfig()[0]}')
            return wlan.ifconfig()[0]
        else:
            print(f"Connection attempt {attempt + 1} failed. Retrying in {retry_delay} seconds...")
            time.sleep(retry_delay)

    print("Failed to connect to WiFi after multiple attempts")
    return False

def wifi_scan():
    wlan = network.WLAN(network.STA_IF)
    wlan.active(True)
    networks = wlan.scan()
    print("Available WiFi networks:")
    for net in networks:
        print(f"SSID: {net[0].decode()}, Signal: {net[3]}dBm")

def sync_time():
    try:
        ntptime.settime()
        print('Time synced with NTP server')
    except:
        print('Failed to sync time')

def sensor_thread():
    i2c_instance = I2C(0, scl=Pin(15), sda=Pin(13), freq=100000)
    scd40_sensor = EnvSensor(i2c_instance, 'SCD40')
    # sht30_sensor = EnvSensor(i2c_instance, 'SHT30')
    scd40_sensor.start_measurement()
    while True:
        try:
            co2, temperature, humidity = scd40_sensor.read_measurement()

            if co2 is not None:
                print(f"CO2: {co2} ppm, Temperature: {temperature:.2f}°C, Humidity: {humidity:.2f}%")
            else:
                print("Sensor reading error")
        except Exception as e:
            print(f"Sensor reading error: {e}")

        time.sleep(MEASUREMENT_RATE_S)

def vent_oneshot_timer_cb(timer):
    vent_pwm_pin.duty(0)
    print('Turn off vent')

def vent_thread():
    vent_oneshot_timer = Timer(0)
    vent_counter = 1
    vent_work_time_ms = VENT_WORK_TIME_MIN * 60 * 1000
    vent_pwm_pin.freq(PWM_FREQUENCY)
    vent_pwm_pin.duty(0)

    while True:
        vent_max_cnt = get_value('vent_max_cnt')

        if get_value('vent_auto'):
            vent_counter = vent_counter + 1
            vent_period = (24 * 60) / vent_max_cnt
            print(f'Set vent_period from json: {vent_period}')

            if vent_counter % vent_period == 0:
                vent_pwm_pin.duty(int(1024 * VENT_PWM_VALUE / 100))
                print('Turn on vent')
                vent_oneshot_timer.init(period=vent_work_time_ms, mode=Timer.ONE_SHOT, callback=vent_oneshot_timer_cb)

        time.sleep(60)

def water_oneshot_timer_cb(timer):
    print('Turn off water')
    water_pin.off()

def water_thread():
    water_oneshot_timer = Timer(1)
    water_counter = 1
    water_work_time_ms = WATER_WORK_TIME_MIN * 60 * 1000
    water_pin.off()

    while True:
        wtr_max_cnt = get_value('wtr_max_cnt')
        day_dur = get_value('day_dur')

        if get_value('water_auto'):
            water_counter = water_counter + 1
            water_cycle_dur = day_dur - 2
            water_period = (water_cycle_dur * 60) / wtr_max_cnt
            print(f'Set water period based on json: {water_period}')

            if check_schedule('water'):
                if water_counter % water_period == 0:
                    print('Turn on water')
                    water_pin.on()
                    water_oneshot_timer.init(period=water_work_time_ms, mode=Timer.ONE_SHOT, callback=water_oneshot_timer_cb)

        time.sleep(60)

def light_thread():
    light_pwm_pin.freq(PWM_FREQUENCY)
    light_pwm_pin.duty(0)
    while True:
        if get_value('light_auto'):
            light_pwm_value_perc = get_value('light_pwm')
            print(f'Set light_pwm from json: {light_pwm_value_perc}')

            if check_schedule('light'):
                print('Turn on light')
                light_pwm_pin.duty(int(1024 * light_pwm_value_perc / 100))

        time.sleep(60)

def time_to_minutes(time_str):
    if time_str is None:
        raise ValueError("Input cannot be None")

    # Optional: Check if the input is in the right format
    if not isinstance(time_str, str) or len(time_str.split(':')) != 2:
        raise ValueError("Input must be a string in the format 'HH:MM'")

    # Proceed to extract hours and minutes
    hours, minutes = map(int, time_str.split(':'))

    return hours * 60 + minutes

def on_message(topic, msg):
    if topic == MQTT_TOPIC_SETTING_DAY_START_HR.encode():
        try:
            day_start_hr = int(msg.decode())
            if 0 <= day_start_hr < 24:
                with data_lock:
                    update_value('day_start_hr', day_start_hr)
                    send_to_mqtt(MQTT_TOPIC_STATUS_DAY_START_HR, str(day_start_hr))
            else:
                raise ValueError("Value should be in range 0-23")
        except ValueError as e:
            print(f"Received invalid value: {msg.decode()}. Error: {e}")
    elif topic == MQTT_TOPIC_SETTING_DAY_START_MIN.encode():
        try:
            day_start_min = int(msg.decode())
            if 0 <= day_start_min < 60:
                with data_lock:
                    update_value('day_start_min', day_start_min)
                    send_to_mqtt(MQTT_TOPIC_STATUS_DAY_START_MIN, str(day_start_min))
            else:
                raise ValueError("Value should be in range 0-60")
        except ValueError as e:
            print(f"Received invalid value: {msg.decode()}. Error: {e}")
    elif topic == MQTT_TOPIC_SETTING_DAY_DUR.encode():
        try:
            day_dur = int(msg.decode())
            if 0 <= day_dur < 24:
                with data_lock:
                    update_value('day_dur', day_dur)
                    send_to_mqtt(MQTT_TOPIC_STATUS_DAY_DUR, str(day_dur))
            else:
                raise ValueError("Value should be in range 0-24")
        except ValueError as e:
            print(f"Received invalid value: {msg.decode()}. Error: {e}")
    elif topic == MQTT_TOPIC_SETTING_TIMEZONE.encode():
        try:
            tzn = int(msg.decode())
            if 0 <= tzn < 24:
                with data_lock:
                    update_value('tzn', tzn)
                    send_to_mqtt(MQTT_TOPIC_STATUS_TIMEZONE, str(tzn))
            else:
                raise ValueError("Value should be in range 0-24")
        except ValueError as e:
            print(f"Received invalid value: {msg.decode()}. Error: {e}")
    elif topic == MQTT_TOPIC_SETTING_LIGHT_PWM.encode():
        try:
            light_pwm = int(msg.decode())
            if 0 <= light_pwm <= 100:
                with data_lock:
                    update_value('light_pwm', light_pwm)
                    send_to_mqtt(MQTT_TOPIC_STATUS_LIGHT_PWM, str(light_pwm))
            else:
                raise ValueError("Value should be in range 0-24")
        except ValueError as e:
            print(f"Received invalid value: {msg.decode()}. Error: {e}")

    elif topic == MQTT_TOPIC_SETTING_WTR_MAX_CNTR.encode():
        try:
            wtr_max_cnt = int(msg.decode())
            if 0 < wtr_max_cnt <= 800:
                with data_lock:
                    update_value('wtr_max_cnt', wtr_max_cnt)
                    send_to_mqtt(MQTT_TOPIC_STATUS_WTR_MAX_CNTR, str(wtr_max_cnt))
            else:
                raise ValueError("Value should be in range 0-800")
        except ValueError as e:
            print(f"Received invalid value: {msg.decode()}. Error: {e}")
    elif topic == MQTT_TOPIC_SETTING_VENT_MAX_CNTR.encode():
        try:
            vent_max_cnt = int(msg.decode())
            if 0 < vent_max_cnt <= 800:
                with data_lock:
                    update_value('vent_max_cnt', vent_max_cnt)
                    client.publish(MQTT_TOPIC_STATUS_VENT_MAX_CNTR, str(vent_max_cnt))
            else:
                raise ValueError("Value should be in range 0-800")
        except ValueError as e:
            print(f"Received invalid value: {msg.decode()}. Error: {e}")
    elif topic == MQTT_TOPIC_SETTING_LIGHT_AUTO.encode():
        try:
            light_auto = int(msg.decode())
            if 0 <= light_auto <= 1:
                with data_lock:
                    update_value('light_auto', light_auto)
                send_to_mqtt(MQTT_TOPIC_STATUS_LIGHT_AUTO, str(light_auto))
            else:
                raise ValueError("Value should be in range 0-1")
        except ValueError as e:
            print(f"Received invalid value: {msg.decode()}. Error: {e}")
    elif topic == MQTT_TOPIC_SETTING_WATER_AUTO.encode():
        try:
            water_auto = int(msg.decode())
            if 0 <= water_auto <= 1:
                with data_lock:
                    update_value('water_auto', water_auto)
                send_to_mqtt(MQTT_TOPIC_STATUS_WATER_AUTO, str(water_auto))
            else:
                raise ValueError("Value should be in range 0-1")
        except ValueError as e:
            print(f"Received invalid value: {msg.decode()}. Error: {e}")
    elif topic == MQTT_TOPIC_SETTING_VENT_AUTO.encode():
        try:
            vent_auto = int(msg.decode())
            if 0 <= vent_auto <= 1:
                with data_lock:
                    update_value('vent_auto', vent_auto)
                send_to_mqtt(MQTT_TOPIC_STATUS_VENT_AUTO, str(vent_auto))
            else:
                raise ValueError("Value should be in range 0-1")
        except ValueError as e:
            print(f"Received invalid value: {msg.decode()}. Error: {e}")
    elif topic == MQTT_TOPIC_WATER_CON.encode():
        try:
            relay_water = int(msg.decode())
            if 0 <= relay_water <= 1:
                with data_lock:
                    if relay_water == 1:
                        print(f"Turn on water")
                        water_pin.on()
                    else:
                        print(f"Turn off water")
                        water_pin.off()
            else:
                raise ValueError("Value should be in range 0-1")
        except ValueError as e:
            print(f"Received invalid value: {msg.decode()}. Error: {e}")
    elif topic == MQTT_TOPIC_VENT_PWM.encode():
        try:
            vent_pwm_value = int(msg.decode())
            if 0 <= vent_pwm_value <= 100:
                with data_lock:
                    vent_pwm_pin.duty(int(1024 * vent_pwm_value / 100))
            else:
                raise ValueError("Value should be in range 0-1")
        except ValueError as e:
            print(f"Received invalid value: {msg.decode()}. Error: {e}")
    elif topic == MQTT_TOPIC_LIGHT_PWM.encode():
        try:
            pwm_light = int(msg.decode())
            if 0 <= pwm_light <= 100:
                with data_lock:
                    light_pwm_pin.duty(int(1024 * pwm_light / 100))
            else:
                raise ValueError("Value should be in range 0-1")
        except ValueError as e:
            print(f"Received invalid value: {msg.decode()}. Error: {e}")
    elif topic == MQTT_TOPIC_REQ_ALL_SETTINGS.encode():
        try:
            request_settings = int(msg.decode())
            if request_settings == 1:
                print(f"Requested settings")
            else:
                raise ValueError("Value should be in range 0-1")
        except ValueError as e:
            print(f"Received invalid value: {msg.decode()}. Error: {e}")
    elif topic == MQTT_TOPIC_REBOOT.encode():
        try:
            request_reboot = int(msg.decode())
            if request_reboot == 1:
                print(f"Requested reboot")
                machine.reset()
            else:
                raise ValueError("Value should be in range 0-1")
        except ValueError as e:
            print(f"Received invalid value: {msg.decode()}. Error: {e}")

def mqtt_thread():
    client = MQTTClient(MQTT_CLIENT_ID, MQTT_HOST, MQTT_PORT)
    client.set_callback(on_message)
    client.connect()
    client.subscribe(MQTT_TOPIC_SETTING_DAY_START_HR)
    client.subscribe(MQTT_TOPIC_SETTING_DAY_START_MIN)
    client.subscribe(MQTT_TOPIC_SETTING_DAY_DUR)
    client.subscribe(MQTT_TOPIC_SETTING_TIMEZONE)
    client.subscribe(MQTT_TOPIC_SETTING_LIGHT_PWM)
    client.subscribe(MQTT_TOPIC_SETTING_WTR_MAX_CNTR)
    client.subscribe(MQTT_TOPIC_SETTING_VENT_MAX_CNTR)
    client.subscribe(MQTT_TOPIC_SETTING_LIGHT_AUTO)
    client.subscribe(MQTT_TOPIC_SETTING_WATER_AUTO)
    client.subscribe(MQTT_TOPIC_SETTING_VENT_AUTO)
    client.subscribe(MQTT_TOPIC_WATER_CON)
    client.subscribe(MQTT_TOPIC_VENT_PWM)
    client.subscribe(MQTT_TOPIC_LIGHT_PWM)
    client.subscribe(MQTT_TOPIC_REQ_ALL_SETTINGS)
    client.subscribe(MQTT_TOPIC_REBOOT)
    print("Connected to MQTT server")
    try:
        while True:
            client.check_msg()
            if len(mqtt_queue) > 0:
                with queue_lock:
                    if len(mqtt_queue) > 0:
                        message = mqtt_queue.popleft()
                        topic = message['topic']
                        payload = message['payload']
                        try:
                            client.publish(topic, payload)
                        except Exception as e:
                            print(f"MQTT send error: {e}")
                            try:
                                client.connect()
                            except:
                                pass
            time.sleep(1)

    except Exception as e:
        print(f"Error in MQTT thread: {e}")

    # while True:
    #     try:
    #         client.check_msg()
    #         with data_lock:
    #             co2 = latest_data["co2"]
    #             temperature = latest_data["temperature"]
    #             humidity = latest_data["humidity"]
    #
    #         if co2 is not None:
    #             # client.publish(MQTT_TOPIC_CO2, str(co2))
    #             # client.publish(MQTT_TOPIC_TEMP, str(temperature))
    #             # client.publish(MQTT_TOPIC_HUM, str(humidity))
    #             print("Data sent to  MQTT")
    #         else:
    #             print("No data to send")
    #     except Exception as e:
    #         print(f"Error in  MQTT thread: {e}")
    #         try:
    #             client.connect()
    #             client.subscribe(MQTT_TOPIC_PWM)
    #         except:
    #             pass

def send_to_mqtt(topic, payload):
    with queue_lock:
        if len(mqtt_queue) < mqtt_queue.maxlen:
            mqtt_queue.append({'topic': topic, 'payload': payload})
            return True
        return False

def connection_manager():

    while True:
        ip_addr = connect_wifi(SSID, PASSWORD)
        if not ip_addr:
            print("Couldn't connect to Wifi...")
        print(f'IP is: {ip_addr}')

        try:
            print(f'Client connect func returned {client.connect()}')
            system_status['mqtt'] = 'connected'
        except:
            print('Unable to connect to MQTT. Check creds')
        if system_status['mqtt'] == 'connected':
            client.set_callback(on_message)
            send_to_mqtt(MQTT_TOPIC_STATUS_SWVER, CURRENT_VERSION)
            send_to_mqtt(MQTT_TOPIC_STATUS_IPADDR, str(ip_addr))
        time.sleep(60)

def init_settings():
    # Set default settings
    if not file_exists(JSON_FILE):
        print("Config file doesn't exist. Creating a new one.")
        save_json({})
        # Set default value after file creation
        update_value('day_start_hr', DEFAULT_DAY_START_HR)
        update_value('day_start_min', DEFAULT_DAY_START_MIN)
        update_value('day_dur', DEFAULT_DAY_DUR)
        update_value('tzn', DEFAULT_TIMEZONE)
        update_value('light_pwm', DEFAULT_LIGHT_PWM)
        update_value('wtr_max_cnt', DEFAULT_WTR_MAX_CNTR)
        update_value('vent_max_cnt', DEFAULT_VENT_MAX_CNTR)
        update_value('light_auto', DEFAULT_LIGHT_AUTO)
        update_value('water_auto', DEFAULT_WATER_AUTO)
        update_value('vent_auto', DEFAULT_VENT_AUTO)

# Main function
def main():

    init_settings()
    connect_wifi(SSID, PASSWORD)
    sync_time()

    # _thread.start_new_thread(light_thread, ())
    # _thread.start_new_thread(sensor_thread, ())
    # _thread.start_new_thread(mqtt_thread, ())
    # _thread.start_new_thread(vent_thread, ())
    # _thread.start_new_thread(water_thread, ())
    _thread.start_new_thread(update_check_task, ())
    # _thread.start_new_thread(connection_manager, ())

    while True:
        gc.collect()
        print(f'\r\nFree memory: {gc.mem_free()}')
        print(f'\r\nAllocated memory: {gc.mem_alloc()}')

        time.sleep(5)

if __name__ == "__main__":
    main()