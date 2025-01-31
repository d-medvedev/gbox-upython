import gc
import time
from machine import SoftI2C, Pin, PWM, reset
from machine import RTC
import _thread
from umqtt.simple import MQTTClient
import network
import ntptime
import utime
import urequests

from envsensor import EnvSensor

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

# Global variables and objects
latest_data = {"co2": None, "temperature": None, "humidity": None}
data_lock = _thread.allocate_lock()
i2c = SoftI2C(scl=Pin(33), sda=Pin(32), freq=100000)
scd40_sensor = EnvSensor(i2c, 'SCD40')
sht30_sensor = EnvSensor(i2c, 'SHT30')


# Wifi functions
def connect_wifi():
    wlan = network.WLAN(network.STA_IF)
    wlan.active(True)
    if not wlan.isconnected():
        print('Connecting to Wi-Fi...')
        wlan.connect(SSID, PASSWORD)
        while not wlan.isconnected():
            pass
    print('Connected to Wi-Fi')
    print('IP address:', wlan.ifconfig()[0])

# def is_wifi_connected():
#     if wlan.isconnected():
#         return True

def sync_time():
    try:
        ntptime.settime()
        print("Time synced with NTP server")
    except:
        print("NTP Sync Error")

def time_sync_task():
    while True:
        sync_time()
        # set_rtc_time()
        utime.sleep(3600)

def sensor_thread():
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

        time.sleep(10)  # Измеряем каждые 5 секунд

# MQTT incoming messages handler
def on_message(topic, msg):
    if topic == MQTT_TOPIC_PWM.encode():
        try:
            new_pwm = int(msg)
            pwm_value = new_pwm
            print(f"Получено новое значение PWM: {new_pwm}")
        except ValueError:
            print(f"Получено некорректное значение PWM: {msg}")

# Функция mqtt_thread
def mqtt_thread():
    client = MQTTClient(MQTT_CLIENT_ID, MQTT_HOST, MQTT_PORT)
    client.set_callback(on_message)
    client.connect()
    client.subscribe(MQTT_TOPIC_PWM)
    print("Подключено к MQTT серверу")

    while True:
        try:
            # Проверяем наличие входящих сообщений
            client.check_msg()

            with data_lock:
                co2 = latest_data["co2"]
                temperature = latest_data["temperature"]
                humidity = latest_data["humidity"]

            if co2 is not None:
                client.publish(MQTT_TOPIC_CO2, str(co2))
                client.publish(MQTT_TOPIC_TEMP, str(temperature))
                client.publish(MQTT_TOPIC_HUM, str(humidity))
                print("Данные отправлены на MQTT сервер")
            else:
                print("Нет данных для отправки")
        except Exception as e:
            print(f"Ошибка в MQTT потоке: {e}")
            try:
                client.connect()
                client.subscribe(MQTT_TOPIC_PWM)
            except:
                pass

        time.sleep(30)

def print_time():
    rtc = RTC()
    date_tuple = rtc.datetime()
    print("Current date and time:")
    print("{:04d}-{:02d}-{:02d} {:02d}:{:02d}:{:02d}".format(
        date_tuple[0], date_tuple[1], date_tuple[2],
        date_tuple[4], date_tuple[5], date_tuple[6]))

# Main function
def main():

    connect_wifi()
    sync_time()

    # Start periodic measurement
    scd40_sensor.start_measurement()
    # _thread.start_new_thread(mqtt_thread, ())

    # Wait for first measurement to be ready (about 5 seconds)
    time.sleep(5)

    while True:
        co2, temperature, humidity = scd40_sensor.read_measurement()
        # _, temp_ext, humid_ext = sht30_sensor.read_measurement()
        print(f"SCD40: CO2: {co2} ppm, Temperature Int: {temperature:.2f}°C, Humidity: {humidity:.2f}%")
        print_time()
        # print(f"SHT30: Temperature Int: {temp_ext:.2f}°C, Humidity: {humid_ext:.2f}%")

        time.sleep(6)

if __name__ == "__main__":
    main()