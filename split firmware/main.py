import gc
import os
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
import neopixel

DEVICE_NAME = 'gbu_dev_3'
CURRENT_VERSION = "0.0.1"
NUM_PIXELS = 7

JSON_FILE = 'config.json'
mqtt_queue = BoundedDeque(maxlen=20)
queue_lock = _thread.allocate_lock()

system = {
    'main': {
        'lamp_pin':  17,  # m5 - 17, cg - 2
        'vent_pin':  25, # m5 - 25, cg - 16
        'pump_pin':  26, # m5 - 26, cg - 17
        'sda_pin':   32, # 32 - m5 = cg
        'scl_pin':   33, # 33 - m5 = cg
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

light_pwm_pin = PWM(Pin(system['main']['lamp_pin']))
vent_pwm_pin = PWM(Pin(system['main']['vent_pin']))
pump_pin = Pin(system['main']['pump_pin'], Pin.OUT)
np = neopixel.NeoPixel(Pin(4),NUM_PIXELS)

def set_color(r, g, b, i):
    # for i in range(NUM_PIXELS):
    np[i] = (r, g, b)
    np.write()

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

def sync_time():
    try:
        ntptime.settime()
        print('Time synced with NTP server')
    except Exception as e:
        print(f'Failed to sync time: Unexpected error - {e}')

def sensor_thread():
    i2c = SoftI2C(scl=Pin(system['main']['scl_pin']),
                  sda=Pin(system['main']['sda_pin']),
                  freq=100000)
    scd40 = EnvSensor(i2c, 'SCD40')
    bh1750 = EnvSensor(i2c, 'BH1750')
    sht30 = EnvSensor(i2c, 'SHT30')

    scd40.start_meas()
    while True:
        try:
            co2, t, rh = scd40.read_meas()
            lux = bh1750.read_meas()
            t_ext, rh_ext = 0, 0 #sht30.read_meas()

            if co2 is not None:
                data = {
                    'co2': co2,
                    't': t,
                    'rh': rh,
                    'lux': lux,
                    't_ext': t_ext,
                    'rh_ext': rh_ext
                }
                send_topic = (system['main']['base_topic'] + system['mqtt_topics_pub']['sns_topic'])
                payload = json.dumps(data)
                send_to_mqtt(send_topic, payload)
                print(f"CO2: {co2} ppm, Temp: {t:.2f}°C, Hum: {rh:.2f}%, Lux: {lux: .1f}")
            else:
                print("Sensor read error")
        except Exception as e:
            print(f"Sensor reading error: {e}")

        time.sleep(system['timers']['meas_rate_s'])

def lamp_thread():
    light_pwm_pin.freq(system['timers']['pwm_freq_hz'])
    light_pwm_pin.duty(0)
    while True:
        if get_value('light_auto'):
            light_pwm_value_perc = get_value('lamp_pwm')
            print(f'\nSet light_pwm from json: {light_pwm_value_perc}')

            if check_schedule('light'):
                print('\nTurn on light')
                light_pwm_pin.duty(int(1024 * light_pwm_value_perc / 100))
                set_color(0, 0, int(light_pwm_value_perc), 4)

        time.sleep(60)

def vent_oneshot_timer_cb(_timer):
    vent_pwm_pin.duty(0)
    print('Turn off vent')

def vent_thread():
    vent_oneshot_timer = Timer(0)
    vent_counter = 1
    vent_work_time_ms = system['timers']['pump_dur_min'] * 60 * 1000
    vent_pwm_pin.freq(system['timers']['pwm_freq_hz'])
    vent_pwm_pin.duty(10)

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

def pump_oneshot_timer_cb(_timer):
    print('Turn off water')
    pump_pin.off()

def pump_thread():
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

def json_replace_errors(msg):
    msg_str = msg.decode('utf-8')

    msg_str = msg_str.replace(':,', ':null,')
    msg_str = msg_str.replace(':}', ':null}')
    msg_str = msg_str.replace(': ,', ':null,')
    msg_str = msg_str.replace(': }', ':null}')
    msg_str = ' '.join(msg_str.split())
    return msg_str

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
        time.sleep(1)


def main():

    init_settings()
    connect_wifi(system['network']['ssid'], system['network']['pass'])
    sync_time()

    _thread.start_new_thread(lamp_thread, ())
    _thread.start_new_thread(sensor_thread, ())
    _thread.start_new_thread(mqtt_thread, ())
    _thread.start_new_thread(vent_thread, ())
    _thread.start_new_thread(pump_thread, ())
    _thread.start_new_thread(update_check_task, ())
    # _thread.start_new_thread(connection_manager, ())

    while True:
        gc.collect()
        print(f'\r\nFree memory: {gc.mem_free()}')
        time.sleep(5)

if __name__ == "__main__":
    main()