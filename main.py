import gc
import os
import re
import time
import struct
import machine
from machine import SoftI2C, Pin, PWM, Timer, RTC
import _thread
from collections import deque
from umqtt.simple import MQTTClient
import network
import ntptime
import utime
import json
import urequests

DEVICE_NAME = 'gbu_dev_3'
CURRENT_VERSION = "5.0.1"

class EnvSensor:
    # Constants
    SCD40_ADDR = 0x62
    SHT30_ADDR = 0x44

    # SCD40 Commands
    SCD40_START_MEAS = 0x21B1
    SCD40_READ_MEAS = 0xEC05
    SCD40_STOP_MEAS = 0x3F86

    # SHT30 Commands
    SHT30_MEASURE_HIGH = 0x2C06

    def __init__(self, i2c, sensor_type):
        self.i2c = i2c
        if sensor_type not in ['SCD40', 'SHT30']:
            raise ValueError("Invalid sensor type. Choose 'SCD40' or 'SHT30'.")
        self.sensor_type = sensor_type
        self.addr = self.SCD40_ADDR if sensor_type == 'SCD40' else self.SHT30_ADDR

    def start_meas(self):
        if self.sensor_type == 'SCD40':
            self._write_command(self.SCD40_START_MEAS)
            time.sleep(5)  # Wait for SCD40 to initialize

    def read_meas(self):
        if self.sensor_type == 'SCD40':
            return self._read_scd40()
        else:
            return self._read_sht30()

    def _read_scd40(self):
        self._write_command(self.SCD40_READ_MEAS)
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

        temp = -45 + 175 * (temp / 65535)
        hum = 100 * (hum / 65535)
        return None, temp, hum  # None for CO2 as SHT30 doesn't measure it

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

JSON_FILE = 'config.json'
mqtt_queue = BoundedDeque(maxlen=20)
queue_lock = _thread.allocate_lock()

system = {
    'main': {
        'lamp_pin':  17,
        'vent_pin':  25,
        'pump_pin':  26,
        'sda':   32,
        'scl':   33,
        'swver': CURRENT_VERSION,
        'base_topic': DEVICE_NAME,
    },
    'default_settings': {
        'day_start_hr':  6,
        'day_start_min': 0,
        'day_dur':       18,
        'tzn':           7,
        'lamp_pwm':      20,
        'wtr_max_cnt':   2,
        'vent_max_cnt':  72,
        'vent_pwm_val':  80,
        'light_auto':    1,
        'wtr_auto':      1,
        'vent_auto':     1,
    },
    'timers': {
        'vent_dur_min': 3,
        'pump_dur_min': 1,
        'meas_rate_s':  30,
        'pwm_freq_hz':  100
    },
    'network': {
        'ssid': "ELTEX-8A08",
        'pass': "GP21204758",
        'ota' : "http://178.236.244.174:5000"
    },
    'mqtt_creds': {
        'host':  "dev.rightech.io",
        'port':  1883,
        'cl_id': DEVICE_NAME,
    },
    'mqtt_topics_sub': {
        'sgs_topic': '/settings',
        'cmd_topic': '/commands',
    },
    'mqtt_topics_pub':         {
        'sts_topic': '/status',
        'sns_topic': '/sensors'
    }
}

def check_for_update():
    ota_url = system['network']['ota']
    swver =  system['main']['swver']
    print(f"\nCurrent sw ver is: {swver}")
    try:
        response = urequests.get(f"{ota_url}/check_update?version={swver}")
        if response.status_code == 200:
            update_info = response.json()
            if update_info.get('update_available', False):
                print("Доступно обновление. Начинаем загрузку...")
                return download_update()
            else:
                print("No need to OTA")
        else:
            print("Error:", response.status_code)
    except Exception as e:
        print("Error:", str(e))
    return False

def download_update():
    ota_url = system['network']['ota']
    try:
        response = urequests.get(f"{ota_url}/update")
        if response.status_code == 200:
            update_files = response.json()
            for file_name, file_content in update_files.items():
                with open(file_name, "w") as f:
                    f.write(file_content)
                print(f"Файл {file_name} обновлен")
            return True
        else:
            print("Error:", response.status_code)
    except Exception as e:
        print("Error:", str(e))
    return False

def update_check_task():
    while True:
        if check_for_update():
            print("OTA OK. Reboot...")
            machine.reset()
        utime.sleep(60)

def check_schedule(equipment):
    rtc = RTC()
    cur_time = rtc.datetime()
    offset_min = get_value('tzn') * 60
    cur_min = (cur_time[4] * 60 + cur_time[5] + offset_min) % 1440
    start_min = get_value('day_start_hr') * 60 + get_value('day_start_min')
    end_min = start_min + get_value('day_dur') * 60
    print(f'start_minutes: {start_min}')
    print(f'end_minutes: {end_min}')
    print(f'current_minutes: {cur_min}')

    if equipment == 'light':

        if start_min <= cur_min < end_min:
            return True
        else:
            return False
    elif equipment == 'water':
        if start_min + 60 <= cur_min < end_min - 60:
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
    i2c = SoftI2C(scl=Pin(system['pinout']['scl']),
                  sda=Pin(system['pinout']['sda']),
                  freq=100000)
    scd40_sensor = EnvSensor(i2c, 'SCD40')
    # sht30_sensor = EnvSensor(i2c_instance, 'SHT30')
    scd40_sensor.start_meas()
    while True:
        try:
            co2, temperature, humidity = scd40_sensor.read_meas()

            if co2 is not None:
                print(f"CO2: {co2} ppm, Temperature: {temperature:.2f}°C, Humidity: {humidity:.2f}%")
            else:
                print("Sensor read error")
        except Exception as e:
            print(f"Sensor reading error: {e}")

        time.sleep(system['timers']['meas_rate_s'])

def lamp_thread():
    light_pwm_pin = PWM(Pin(system['main']['lamp_pin']))
    light_pwm_pin.freq(system['timers']['pwm_freq_hz'])
    light_pwm_pin.duty(0)
    while True:
        if get_value('light_auto'):
            light_pwm_value_perc = get_value('lamp_pwm')
            print(f'\nSet light_pwm from json: {light_pwm_value_perc}')

            if check_schedule('light'):
                print('\nTurn on light')
                light_pwm_pin.duty(int(1024 * light_pwm_value_perc / 100))

        time.sleep(60)

def vent_oneshot_timer_cb(timer):
    vent_pwm_pin.duty(0)
    print('Turn off vent')

def vent_thread():
    global vent_pwm_pin
    vent_pwm_pin = PWM(Pin(system['main']['vent_pin']))
    vent_oneshot_timer = Timer(0)
    vent_counter = 1
    vent_work_time_ms = system['timers']['pump_dur_min'] * 60 * 1000
    vent_pwm_pin.freq(system['timers']['pwm_freq_hz'])
    vent_pwm_pin.duty(0)

    while True:
        vent_max_cnt = get_value('vent_max_cnt')

        if get_value('vent_auto'):
            vent_counter = vent_counter + 1
            vent_period = (24 * 60) / vent_max_cnt
            print(f'Set vent_period from json: {vent_period}')

            if vent_counter % vent_period == 0:
                vent_pwm_pin.duty(int(1024 * system['default_settings']['vent_pwm_val'] / 100))
                print('Turn on vent')
                vent_oneshot_timer.init(period=vent_work_time_ms,
                                        mode=Timer.ONE_SHOT,
                                        callback=vent_oneshot_timer_cb)
        time.sleep(60)

def pump_oneshot_timer_cb(timer):
    print('Turn off water')
    pump_pin.off()

def pump_thread():
    global pump_pin
    pump_pin = Pin(system['main']['pump_pin'])
    water_oneshot_timer = Timer(1)
    water_counter = 1
    water_work_time_ms = system['timers']['pump_dur_min'] * 60 * 1000
    pump_pin.off()

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
                    pump_pin.on()
                    water_oneshot_timer.init(period=water_work_time_ms,
                                             mode=Timer.ONE_SHOT,
                                             callback=pump_oneshot_timer_cb)

        time.sleep(60)

def send_to_mqtt(topic, payload):
    with queue_lock:
        if len(mqtt_queue) < mqtt_queue.maxlen:
            mqtt_queue.append({'topic': topic, 'payload': payload})
            return True
        return False

def init_settings():
    # Set default settings
    if not file_exists(JSON_FILE):
        print("Config file doesn't exist. Creating a new one.")
        save_json({})

        for key, value in system['default_settings'].items():
            update_value(key, value)

def validate_value(key, value):
    ranges = {'day_start_hr': (0,24), 'day_start_min': (0,59), 'day_dur': (0,24),
              'tzn': (-12,12), 'lamp_pwm': (0,100), 'wtr_max_cnt': (1,5),
              'vent_max_cnt': (1,100), 'vent_pwm_val': (0,100),
              'light_auto': (0,1), 'wtr_auto': (0,1), 'vent_auto': (0,1)}
    return (ranges[key][0] <= value <= ranges[key][1],
            None if ranges[key][0] <= value <= ranges[key][1]
            else f"Value {value} is out of range ({ranges[key][0]}-{ranges[key][1]})")

def on_message(topic, msg):
    if topic == (system['main']['base_topic'] + system['mqtt_topics_sub']['sgs_topic']).encode():
        try:
            msg_str = msg.decode('utf-8')

            msg_str = msg_str.replace(':,', ':null,')
            msg_str = msg_str.replace(':}', ':null}')
            msg_str = msg_str.replace(': ,', ':null,')
            msg_str = msg_str.replace(': }', ':null}')
            msg_str = ' '.join(msg_str.split())

            try:
                new_config = json.loads(msg_str)
            except json.JSONDecodeError as e:
                print(f"Error: Invalid JSON format - {e}")
                return

            current_config = {}

            try:
                with open('config.json', 'r') as f:
                    current_config = json.load(f)
            except Exception as e:
                print(f"Warning: Could not load current config - {e}")

            updates = {}
            skipped = []
            validation_errors = []

            for key in system['default_settings'].keys():
                if key in new_config:
                    value = new_config[key]
                    if value is not None:
                        try:
                            if isinstance(value, (int, float)):
                                is_valid, error_msg = validate_value(key, int(value))
                                if is_valid:
                                    updates[key] = int(value)
                                else:
                                    validation_errors.append(f"{key}: {error_msg}")
                                    skipped.append(key)
                            else:
                                skipped.append(key)
                        except (ValueError, TypeError):
                            skipped.append(key)
                    else:
                        skipped.append(key)
                else:
                    skipped.append(key)

            if updates:
                current_config.update(updates)

                try:
                    with open('config.json', 'w') as f:
                        json.dump(current_config, f)
                    print(f"\nSuccessfully updated {len(updates)} settings:")
                    for key, value in sorted(updates.items()):
                        print(f"- {key}: {value}")
                    send_topic = (system['main']['base_topic'] + system['mqtt_topics_pub']['sts_topic'])
                    payload = json_str = json.dumps(current_config)
                    send_to_mqtt(send_topic, payload)
                except Exception as e:
                    print(f"Error saving config: {e}")

                if validation_errors:
                    print("\nValidation errors:")
                    for error in validation_errors:
                        print(f"- {error}")

                if skipped:
                    print(f"\nSkipped {len(skipped)} invalid or empty values:")
                    for key in sorted(skipped):
                        if key not in [err.split(':')[0] for err in validation_errors]:
                            print(f"- {key}")
            else:
                print("No valid updates received")
                if validation_errors:
                    print("\nValidation errors:")
                    for error in validation_errors:
                        print(f"- {error}")

        except Exception as e:
            print(f"Error processing message: {e}")

def mqtt_thread():

    client = MQTTClient(system['mqtt_creds']['cl_id'],
                        system['mqtt_creds']['host'],
                        system['mqtt_creds']['port'])

    client.set_callback(on_message)
    client.connect()
    print("MQTT connected!")

    for topic in system['mqtt_topics_sub'].values():
        topic = system['main']['base_topic'] + topic
        client.subscribe(topic)
        print(f'Subscribed to : {topic}')
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
                            # try:
                            #     client.connect()
                            # except:
                            #     pass
            time.sleep(1)

    except Exception as e:
        print(f"Error in MQTT thread: {e}")

def main():

    init_settings()
    connect_wifi(system['network']['ssid'], system['network']['pass'])
    sync_time()

    _thread.start_new_thread(lamp_thread, ())
    # _thread.start_new_thread(sensor_thread, ())
    _thread.start_new_thread(mqtt_thread, ())
    _thread.start_new_thread(vent_thread, ())
    _thread.start_new_thread(pump_thread, ())
    _thread.start_new_thread(update_check_task, ())
    # _thread.start_new_thread(connection_manager, ())

    while True:
        gc.collect()
        print(f'\r\nFree memory: {gc.mem_free()}')
        print(f'\r\nAllocated memory: {gc.mem_alloc()}')

        time.sleep(5)

if __name__ == "__main__":
    main()