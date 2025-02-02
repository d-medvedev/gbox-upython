import gc
import os
import time

from machine import SoftI2C, Pin, PWM, reset
from machine import RTC
import _thread
from umqtt.simple import MQTTClient
import network
import ntptime
import utime
import json
import urequests

from envsensor import EnvSensor

# Pin mapping
LED_PIN = 2


# Настройки Wi-Fi
SSID        = "ELTEX-8A08"
PASSWORD    = "GP21204758"

# Server settings
SERVER_URL      = "http://192.168.1.188:5000"
CURRENT_VERSION = "1.1.0"

# MQTT settings
MQTT_HOST       = "dev.rightech.io"
MQTT_PORT       = 1883
MQTT_CLIENT_ID  = "gbu_dev_3"

# Sensors topics (publish)
MQTT_TOPIC_TEMP     = "gbu_dev_3/Scd40/t"
MQTT_TOPIC_HUM      = "gbu_dev_3/Scd40/rh"
MQTT_TOPIC_CO2      = "gbu_dev_3/Scd40/cd"
MQTT_TOPIC_TEMP_EXT = "gbu_dev_3/Sht30/t_ext"
MQTT_TOPIC_HUM_EXT  = "gbu_dev_3/Sht30/h_ext"
MQTT_TOPIC_LUX      = "gbu_dev_3/Bh1750/lux"
MQTT_TOPIC_PWM      = "gbu_dev_3/relay/light_pwm"

# Settings topics (subscribe)
MQTT_TOPIC_SETTING_DAY_START_HR  = "gbu_dev_3/settings/day_start_hr"
MQTT_TOPIC_SETTING_DAY_START_MIN = "gbu_dev_3/settings/day_start_min"
MQTT_TOPIC_SETTING_DAY_DUR       = "gbu_dev_3/settings/day_dur"
MQTT_TOPIC_SETTING_TIMEZONE      = "gbu_dev_3/settings/tzn"
MQTT_TOPIC_SETTING_WTR_MAX_CNTR  = "gbu_dev_3/wtr_max_cnt"
MQTT_TOPIC_SETTING_WTR_DUR       = "gbu_dev_3/settings/wtr_dur"
MQTT_TOPIC_SETTING_WTR_OFFSET    = "gbu_dev_3/settings/wtr_offset"
MQTT_TOPIC_SETTING_GLOB_AUTO     = "gbu_dev_3/settings/global_auto"
MQTT_TOPIC_SETTING_LIGHT_AUTO    = "gbu_dev_3/settings/light_auto"
MQTT_TOPIC_SETTING_CO2_AUTO      = "gbu_dev_3/settings/co2_auto"
MQTT_TOPIC_SETTING_WATER_AUTO    = "gbu_dev_3/settings/water_auto"

# Commands topic (subscribe)
MQTT_TOPIC_WATER_CON        = "gbu_dev_3/relay/water"
MQTT_TOPIC_VENT_CON         = "gbu_dev_3/relay/vent"
MQTT_TOPIC_LIGHT_CON        = "gbu_dev_3/relay/light"
MQTT_TOPIC_LIGHT_PWM        = "gbu_dev_3/relay/light_pwm"
MQTT_TOPIC_REQ_ALL_SETTINGS = "gbu_dev_3/cmd/req_all"
MQTT_TOPIC_REBOOT           = "gbu_dev_3/cmd/reboot"

# Status topics
MQTT_TOPIC_STATUS_DAY_START_HR  = "gbu_dev_3/status/day_start_hr"
MQTT_TOPIC_STATUS_DAY_START_MIN = "gbu_dev_3/status/day_start_min"
MQTT_TOPIC_STATUS_DAY_DUR       = "gbu_dev_3/status/day_dur"
MQTT_TOPIC_STATUS_TIMEZONE      = "gbu_dev_3/status/tzn"

# Global variables and objects
client = MQTTClient(MQTT_CLIENT_ID, MQTT_HOST, MQTT_PORT)
latest_data = {"co2": None, "temperature": None, "humidity": None}
data_lock = _thread.allocate_lock()
i2c = SoftI2C(scl=Pin(33), sda=Pin(32), freq=100000)
scd40_sensor = EnvSensor(i2c, 'SCD40')
sht30_sensor = EnvSensor(i2c, 'SHT30')
JSON_FILE = 'config.json'

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

# Wifi functions
def connect_wifi(ssid, password, max_attempts=5, retry_delay=5):
    wlan = network.WLAN(network.STA_IF)
    wlan.active(True)

    if wlan.isconnected():
        print("Already connected to WiFi")
        return True

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
            print("WiFi connected. Network config:", wlan.ifconfig())
            return True
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

def set_rtc():
    try:
        ntptime.settime()
        print('Time synced with NTP server')
    except:
        print('Failed to sync time')

def sensor_thread():
    scd40_sensor.start_measurement()
    while True:
        try:
            co2, temperature, humidity = scd40_sensor.read_measurement()

            if co2 is not None:
                with data_lock:
                    latest_data["co2"] = co2
                    latest_data["temperature"] = temperature
                    latest_data["humidity"] = humidity
                print(f"CO2: {co2} ppm, Temperature: {temperature:.2f}°C, Humidity: {humidity:.2f}%")
            else:
                print("Sensor reading error")
        except Exception as e:
            print(f"Sensor reading error: {e}")

        time.sleep(5)  # Измеряем каждые 5 секунд

def time_to_minutes(time_str):
    hours, minutes = map(int, time_str.split(':'))
    return hours * 60 + minutes

# Light control class
class LightControl:
    def __init__(self, pin):
        # self.led = machine.Pin(pin, machine.Pin.OUT)
        # self.config = load_config()
        self.rtc = RTC()

    def update_config(self, start_time, end_time):
        update_value('start_time', '06:00')
        update_value('end_time', '00:00')

    def check_schedule(self):
        current_time = self.rtc.datetime()
        offset_minutes = get_value('time_zone') * 60
        current_minutes = (current_time[4] * 60 + current_time[5] + offset_minutes) % 1440
        start_minutes = time_to_minutes(get_value('start_time'))
        end_minutes = time_to_minutes(get_value('end_time'))
        print(f'start_minutes: {start_minutes}')
        print(f'end_minutes: {end_minutes}')
        print(f'current_minutes: {current_minutes}')
        if start_minutes <= current_minutes < end_minutes:
            # self.led.on()
            print('Turn on light')
        else:
            # self.led.off()
            print('Turn off light')

    def run(self):
        while True:
            self.check_schedule()
            time.sleep(60)  # Check every minute

# MQTT incoming messages handler
def on_message(topic, msg):
    if topic == MQTT_TOPIC_SETTING_DAY_START_HR.encode():
        try:
            day_start_hr = int(msg.decode())
            if 0 <= day_start_hr < 24:
                update_value('day_start_hr', day_start_hr)
                print(f"Время начала дня обновлено: {day_start_hr}")
            else:
                raise ValueError("Значение должно быть от 0 до 23")
        except ValueError as e:
            print(f"Получено некорректное значение: {msg.decode()}. Ошибка: {e}")
    elif topic == MQTT_TOPIC_SETTING_DAY_START_MIN.encode():
        try:
            day_start_min = int(msg.decode())
            if 0 <= day_start_min < 60:
                update_value('day_start_min', day_start_min)
                print(f"Время начала дня обновлено: {day_start_min}")
                client.publish(MQTT_TOPIC_STATUS_DAY_START_MIN, day_start_min)
            else:
                raise ValueError("Значение должно быть от 0 до 23")
        except ValueError as e:
            print(f"Получено некорректное значение: {msg.decode()}. Ошибка: {e}")

def mqtt_thread():
    client.set_callback(on_message)
    client.connect()
    client.subscribe(MQTT_TOPIC_PWM)
    print("Connected to MQTT server")

    while True:
        try:
            client.check_msg()

            with data_lock:
                co2 = latest_data["co2"]
                temperature = latest_data["temperature"]
                humidity = latest_data["humidity"]

            if co2 is not None:
                client.publish(MQTT_TOPIC_CO2, str(co2))
                client.publish(MQTT_TOPIC_TEMP, str(temperature))
                client.publish(MQTT_TOPIC_HUM, str(humidity))
                print("Data sent to  MQTT")
            else:
                print("No data to send")
        except Exception as e:
            print(f"Error in  MQTT thread: {e}")
            try:
                client.connect()
                client.subscribe(MQTT_TOPIC_PWM)
            except:
                pass

        time.sleep(30)


# Main function
def main():

    # Set default settings
    if not file_exists(JSON_FILE):
        print("Config file doesn't exist. Creating a new one.")
        save_json({})

    update_value('day_start_hr', 6)
    update_value('day_start_min', 0)
    update_value('day_duration', 18)
    update_value('time_zone', 7)

    wifi_scan()

    if not connect_wifi(SSID, PASSWORD):
        print("Couldn't connect to Wifi...")

    set_rtc()

    # _thread.start_new_thread(sensor_thread, ())
    _thread.start_new_thread(mqtt_thread, ())

    light_control = LightControl(LED_PIN)
    light_control.run()

    while True:
       time.sleep(5)

if __name__ == "__main__":
    main()