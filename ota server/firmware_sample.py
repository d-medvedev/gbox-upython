from machine import I2C, Pin, PWM, reset
from umqtt.simple import MQTTClient
import urequests
import _thread
import network
import ntptime
import struct
import utime
import time
import json
import uos
import gc
import os

# Настройки Wi-Fi
SSID = "redmadrobot"
PASSWORD = "a1b2c3d4e5"
#SSID = "ELTEX-8A08"
#PASSWORD = "GP21204758"

# Настройки сервера обновлений
SERVER_URL = "http://178.236.244.174:5000"
CURRENT_VERSION = "1.1.1"  # Текущая версия прошивки

# Настройки I2C для датчика SCD40
i2c_scd40 = I2C(0, scl=Pin(15), sda=Pin(13), freq=100000)
SCD40_ADDR = 0x62

# Настройки I2C для RTC
i2c_rtc = I2C(1, scl=Pin(12), sda=Pin(11), freq=100000)
BM8563_ADDR = 0x51

# Настройки MQTT
MQTT_HOST = "dev.rightech.io"
MQTT_PORT = 1883
MQTT_CLIENT_ID = "upython"
MQTT_TOPIC_TEMP = "upython/Scd40/t"
MQTT_TOPIC_HUM = "upython/Scd40/rh"
MQTT_TOPIC_CO2 = "upython/Scd40/cd"
MQTT_TOPIC_PWM = "upython/relay/light_pwm"

# Настройка пинов для энкодера
encoder_a = Pin(40, Pin.IN, Pin.PULL_UP)
encoder_b = Pin(41, Pin.IN, Pin.PULL_UP)

# Настройка пина для пищалки
buzzer = PWM(Pin(3))
buzzer.duty(0)  # Выключаем пищалку

# Глобальные переменные
latest_data = {"co2": None, "temperature": None, "humidity": None}
data_lock = _thread.allocate_lock()
encoder_value = 0
last_a = encoder_a.value()
pwm_value = 0  # Переменная для хранения значения PWM


class DeviceData:
    def __init__(self):
        self.device_name = None
        self.object_count = 0
        self.objects = []

    def add_object(self, channel, start_time, duration, periodicity_type, periodicity, season_start_date, season_end_date, seasons_count):
        obj_data = {
            'channel': channel,
            'start_time': start_time,
            'duration': duration,
            'periodicity_type': periodicity_type,
            'periodicity': periodicity,
            'season_start_date': season_start_date,
            'season_end_date': season_end_date,
            'seasons_count': seasons_count
        }
        self.objects.append(obj_data)
        self.object_count += 1

    def to_json(self):
        return json.dumps({
            'device_name': self.device_name,
            'object_count': self.object_count,
            'objects': self.objects
        })

    @staticmethod
    def from_json(json_str):
        data = json.loads(json_str)
        device_data = DeviceData()
        device_data.device_name = data.get('device_name')
        device_data.object_count = data.get('object_count')
        device_data.objects = data.get('objects', [])
        return device_data

# Сохранение данных в NVS
def save_device_data(device_data):
    with open('/nvs_device_data.json', 'w') as f:
        f.write(device_data.to_json())

# Загрузка данных из NVS
def load_device_data():
    try:
        with open('/nvs_device_data.json', 'r') as f:
            data = f.read()
        return DeviceData.from_json(data)
    except OSError:
        return DeviceData()
        
# Функции для SCD40
def crc8(data):
    crc = 0xFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 0x80:
                crc = (crc << 1) ^ 0x31
            else:
                crc = crc << 1
    return crc & 0xFF

def scd40_write_command(command):
    i2c_scd40.writeto(SCD40_ADDR, struct.pack(">H", command))

def scd40_read_data(length):
    return i2c_scd40.readfrom(SCD40_ADDR, length)

def initialize_scd40():
    scd40_write_command(0x21B1)  # Start periodic measurement
    print("Инициализация SCD40...")
    time.sleep(5)  # Даем датчику время на первое измерение

def read_scd40_data():
    scd40_write_command(0xEC05)  # Read measurement
    time.sleep(0.1)  # Wait for data to be ready
    data = scd40_read_data(9)
    if len(data) == 9:
        co2, co2_crc = struct.unpack(">HB", data[:3])
        temp, temp_crc = struct.unpack(">HB", data[3:6])
        hum, hum_crc = struct.unpack(">HB", data[6:9])
        
        if (crc8(data[:2]) == co2_crc and
            crc8(data[3:5]) == temp_crc and
            crc8(data[6:8]) == hum_crc):
            
            co2 = co2
            temperature = -45 + 175 * temp / 65535
            humidity = 100 * hum / 65535
            
            return co2, temperature, humidity
    return None, None, None

# Функции для BM8563 (RTC)
def bcd2dec(bcd):
    return (((bcd & 0xf0) >> 4) * 10 + (bcd & 0x0f))

def dec2bcd(dec):
    tens, units = divmod(dec, 10)
    return (tens << 4) + units

def get_time():
    data = i2c_rtc.readfrom_mem(BM8563_ADDR, 0x02, 7)
    return [bcd2dec(data[6]) + 2000, bcd2dec(data[5] & 0x1F), bcd2dec(data[3] & 0x3F), 
            bcd2dec(data[2] & 0x3F), bcd2dec(data[1] & 0x7F), bcd2dec(data[0] & 0x7F)]

def set_time(dt):
    data = bytearray(7)
    data[0] = dec2bcd(dt[5]) & 0x7F  # секунды
    data[1] = dec2bcd(dt[4]) & 0x7F  # минуты
    data[2] = dec2bcd(dt[3]) & 0x3F  # часы
    data[3] = dec2bcd(dt[2]) & 0x3F  # день
    data[4] = dt[6] & 0x07  # день недели (0-6)
    data[5] = dec2bcd(dt[1]) & 0x1F  # месяц
    data[6] = dec2bcd(dt[0] - 2000)  # год
    i2c_rtc.writeto_mem(BM8563_ADDR, 0x02, data)

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

def set_rtc_time():
    current_time = utime.localtime()
    rtc_time = get_time()
    
    rtc_timestamp = utime.mktime((rtc_time[0], rtc_time[1], rtc_time[2], rtc_time[3], rtc_time[4], rtc_time[5], 0, 0))
    current_timestamp = utime.mktime(current_time)
    
    if abs(current_timestamp - rtc_timestamp) > 10:
        print("Устанавливаем новое время на RTC")
        set_time(current_time)
    else:
        print("Время на RTC актуально")

# Функции для энкодера и пищалки
def beep(frequency, duration):
    buzzer.freq(frequency)
    buzzer.duty(512)  # 50% громкости
    utime.sleep_ms(duration)
    buzzer.duty(0)

def handle_interrupt(pin):
    global encoder_value, last_a
    a = encoder_a.value()
    b = encoder_b.value()
    if a != last_a:
        if b != a:
            encoder_value += 1
            beep(2000, 50)  # Высокий звук при вращении по часовой стрелке
        else:
            encoder_value -= 1
            beep(1000, 50)  # Низкий звук при вращении против часовой стрелки
        last_a = a
        print("Значение энкодера:", encoder_value)

# Настройка прерывания для пина A энкодера
encoder_a.irq(trigger=Pin.IRQ_RISING | Pin.IRQ_FALLING, handler=handle_interrupt)

# Задачи для потоков
def sensor_thread():
    initialize_scd40()
    while True:
        try:
            co2, temperature, humidity = read_scd40_data()
            if co2 is not None:
                with data_lock:
                    latest_data["co2"] = co2
                    latest_data["temperature"] = temperature
                    latest_data["humidity"] = humidity
                print(f"CO2: {co2} ppm, Температура: {temperature:.2f}°C, Влажность: {humidity:.2f}%")
            else:
                print("Ошибка чтения данных")
        except Exception as e:
            print(f"Ошибка чтения данных: {e}")
        
        time.sleep(10)  # Измеряем каждые 5 секунд

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

        time.sleep(30)  # Отправляем данные и проверяем входящие сообщения каждые 10 секунд


def time_sync_task():
    while True:
        sync_time()
        set_rtc_time()
        utime.sleep(3600)  # Синхронизация каждый час

# Функции для обновления прошивки
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

# Задача для проверки обновлений
def update_check_task():
    while True:
        if check_for_update():
            print("Обновление успешно загружено. Перезагрузка...")
            reset()
        utime.sleep(60)  # Проверка обновлений каждый час

# Функция для вывода информации об использовании памяти
def memory_info():
    while True:
        # Получаем текущую информацию о памяти
        gc.collect()  # Запускаем сборщик мусора перед проверкой состояния памяти
        free_mem = gc.mem_free()
        alloc_mem = gc.mem_alloc()
        
        # Выводим информацию в терминал
        print("Используемая оперативная память (RAM): {} байт".format(alloc_mem))
        print("Свободная оперативная память (RAM): {} байт".format(free_mem))
        
        # Получаем информацию о размере флеш-памяти
        stat = os.statvfs('/')
        total_flash = stat[0] * stat[2]
        
        # Выводим размер прошивки
        print("Общий размер флеш-памяти (Flash): {} байт".format(total_flash))
        time.sleep(60)  # Вывод информации каждую минуту

def parse_date(date_str):
    year, month, day = map(int, date_str.split("-"))
    if year < 1970:
        raise ValueError("Год должен быть >= 1970")
    if month < 1 or month > 12:
        raise ValueError("Месяц должен находиться в диапазоне от 1 до 12 включительно.")
    if day < 1 or day > 31:
        raise ValueError("День должен быть в диапазоне от 1 до 31 включительно.")
    return (year * 3600) + (minutes * 60)

def parse_date(date_str):
    year, month, day = map(int, date_str.split("-"))
    return ((year - 1970) + (month * 36725) + (day * 36525)) // 36525

def parse_time(time_str):
    hours, minutes = map(int, time_str.split(":"))
    return (hours * 3600) + (minutes * 60)
                         
def find_matching_objects(device_data, current_timestamp):
    matching_objects = []

    for i, obj in enumerate(device_data.objects):
        start_time = parse_time(obj["start_time"])
        season_start_date = parse_date(obj["season_start_date"])
        season_end_date = parse_date(obj["season_end_date"]) + 86400  # Добавляем 1 день, чтобы включить сезон_конец_дата

        if (
            start_time == check_time
            and season_start_date <= current_timestamp <= season_end_date
        ):
            matching_objects.append((i, obj["channel"], obj["duration"]))

    return matching_objects

def convert_time_from_get_time(time):
    year, month, day, hours, minutes, seconds = time
    date_str = f"{year}-{month:02}-{day}"
    time_str = f"{hours:02}:{minutes:02}"
    return date_str, time_str
        
    
# Основная функция
def main():
    connect_wifi()
    sync_time()
    set_rtc_time()

    if check_for_update():
        print("Обновление успешно загружено. Перезагрузка...")
        reset()
    
    _thread.start_new_thread(sensor_thread, ())
    _thread.start_new_thread(mqtt_thread, ())
    _thread.start_new_thread(time_sync_task, ())
    _thread.start_new_thread(update_check_task, ())
    _thread.start_new_thread(memory_info, ())
    
    while True:
        # Вы можете добавить здесь дополнительную логику для использования pwm_value
        print(f"Текущее значение PWM: {pwm_value}")
        gc.collect()  # Регулярно вызываем сборщик мусора
        time.sleep(60)  # Выводим значение PWM каждую минуту
        

if __name__ == "__main__":

# пример как прочитать данные из энергонезависимой памяти
    loaded_device_data = load_device_data()
    print(f"Loaded device name: {loaded_device_data.device_name}")
    for i, obj in enumerate(loaded_device_data.objects):
        print(f"Object {i+1}:")
        print(f"\tChannel: {obj['channel']}")
        print(f"\tStart time: {obj['start_time']}")
        print(f"\tDuration: {obj['duration']}")
        print(f"\tPeriodicity type: {obj['periodicity_type']}, Periodicity: {obj['periodicity']}")
        print(f"\tSeason start date: {obj['season_start_date']}")
        print(f"\tSeason end date: {obj['season_end_date']}")
        print(f"\tSeasons count: {obj['seasons_count']}")

    main()
    
    
    

