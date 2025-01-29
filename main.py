import gc
import time
import struct
from machine import SoftI2C, Pin, PWM, reset
import _thread
from umqtt.simple import MQTTClient
import network
import ntptime
from scd40 import SCD40
import utime
import urequests

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

# Глобальные переменные
data_lock = _thread.allocate_lock()

# Функции для Wi-Fi и синхронизации времени
def connect_wifi():
    wlan = network.WLAN(network.STA_IF)
    wlan.active(True)
    if not wlan.isconnected():
        print('Подключение к Wi-Fi...')
        wlan.connect(SSID, PASSWORD)
        while not wlan.isconnected():
            pass
    print('Подключено к Wi-Fi')
    print('IP адрес:', wlan.ifconfig()[0])

def sync_time():
    try:
        ntptime.settime()
        print("Время синхронизировано с NTP сервером")
    except:
        print("Ошибка синхронизации времени")


# def sensor_thread():
#     while True:
#         try:
#             co2, temperature, humidity = sensor.read_measurement()
#
#             if co2 is not None:
#                 with data_lock:
#                     latest_data["co2"] = co2
#                     latest_data["temperature"] = temperature
#                     latest_data["humidity"] = humidity
#                 print(f"CO2: {co2} ppm, Температура: {temperature:.2f}°C, Влажность: {humidity:.2f}%")
#             else:
#                 print("Ошибка чтения данных")
#         except Exception as e:
#             print(f"Ошибка чтения данных: {e}")
#
#         time.sleep(10)  # Измеряем каждые 5 секунд

# Функция обратного вызова для обработки входящих MQTT сообщений
def on_message(topic, msg):
    global pwm_value
    if topic == MQTT_TOPIC_PWM.encode():
        try:
            new_pwm = int(msg)
            pwm_value = new_pwm
            print(f"Получено новое значение PWM: {new_pwm}")
        except ValueError:
            print(f"Получено некорректное значение PWM: {msg}")

# Функция mqtt_thread
# def mqtt_thread():
#     client = MQTTClient(MQTT_CLIENT_ID, MQTT_HOST, MQTT_PORT)
#     client.set_callback(on_message)
#     client.connect()
#     client.subscribe(MQTT_TOPIC_PWM)
#     print("Подключено к MQTT серверу")
#
#     while True:
#         try:
#             # Проверяем наличие входящих сообщений
#             client.check_msg()
#
#             with data_lock:
#                 co2 = latest_data["co2"]
#                 temperature = latest_data["temperature"]
#                 humidity = latest_data["humidity"]
#
#             if co2 is not None:
#                 client.publish(MQTT_TOPIC_CO2, str(co2))
#                 client.publish(MQTT_TOPIC_TEMP, str(temperature))
#                 client.publish(MQTT_TOPIC_HUM, str(humidity))
#                 print("Данные отправлены на MQTT сервер")
#             else:
#                 print("Нет данных для отправки")
#         except Exception as e:
#             print(f"Ошибка в MQTT потоке: {e}")
#             try:
#                 client.connect()
#                 client.subscribe(MQTT_TOPIC_PWM)
#             except:
#                 pass
#
#         time.sleep(30)  # Отправляем данные и проверяем входящие сообщения каждые 10 секунд

# Main function
def main():
    # Initialize I2C - adjust pins as necessary for your board
    i2c = SoftI2C(scl=Pin(33), sda=Pin(32), freq=50000)

    # Create SCD40 object
    scd40 = SCD40(i2c)

    # Start periodic measurement
    scd40.start_periodic_measurement()

    # Wait for first measurement to be ready (about 5 seconds)
    time.sleep(5)

    while True:
        co2, temperature, humidity = scd40.read_measurement()
        print(f"CO2: {co2} ppm, Temperature: {temperature:.2f}°C, Humidity: {humidity:.2f}%")
        time.sleep(7)

if __name__ == "__main__":
    main()