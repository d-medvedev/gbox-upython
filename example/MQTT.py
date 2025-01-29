import tkinter as tk
from tkinter import ttk
import paho.mqtt.client as mqtt

# Параметры подключения к MQTT серверу
MQTT_HOST = "dev.rightech.io"
MQTT_PORT = 1883
MQTT_CLIENT_ID = "mqtt-dev"

# Топики для подписки
TOPIC_TEMPERATURE = "upython/Scd40/t"
TOPIC_HUMIDITY = "upython/Scd40/rh"
TOPIC_CO2 = "upython/Scd40/cd"
TOPIC_LIGHT_PWM = "upython/relay/light_pwm"

# Глобальные переменные для хранения данных
temperature = "N/A"
humidity = "N/A"
co2 = "N/A"

# Функция обновления данных в GUI
def update_gui():
    temp_label.config(text=f"Температура: {temperature}")
    humidity_label.config(text=f"Влажность: {humidity}")
    co2_label.config(text=f"CO2: {co2}")
    root.after(1000, update_gui)  # Обновление каждую секунду

# Функции обратного вызова для MQTT
def on_connect(client, userdata, flags, rc, properties=None):
    print(f"Connected with result code {rc}")
    # Подписка на топики после подключения
    client.subscribe(TOPIC_TEMPERATURE)
    client.subscribe(TOPIC_HUMIDITY)
    client.subscribe(TOPIC_CO2)

def on_message(client, userdata, msg):
    global temperature, humidity, co2
    if msg.topic == TOPIC_TEMPERATURE:
        temperature = msg.payload.decode()
    elif msg.topic == TOPIC_HUMIDITY:
        humidity = msg.payload.decode()
    elif msg.topic == TOPIC_CO2:
        co2 = msg.payload.decode()

# Функция для отправки данных по нажатию кнопки
def send_data():
    data = entry.get()
    client.publish(TOPIC_LIGHT_PWM, data)
    entry.delete(0, tk.END)

# Настройка MQTT клиента
client = mqtt.Client(client_id=MQTT_CLIENT_ID)

# Установка версии протокола MQTT
client._protocol = mqtt.MQTTv5

# Установка callback-функций
client.on_connect = on_connect
client.on_message = on_message

# Подключение к MQTT серверу
client.connect(MQTT_HOST, MQTT_PORT, 60)

# Создание GUI
root = tk.Tk()
root.title("MQTT Monitor")

# Создание и размещение элементов интерфейса
temp_label = ttk.Label(root, text="Температура: N/A")
temp_label.pack(pady=5)

humidity_label = ttk.Label(root, text="Влажность: N/A")
humidity_label.pack(pady=5)

co2_label = ttk.Label(root, text="CO2: N/A")
co2_label.pack(pady=5)

entry = ttk.Entry(root)
entry.pack(pady=5)

send_button = ttk.Button(root, text="Отправить", command=send_data)
send_button.pack(pady=5)

# Запуск MQTT клиента в отдельном потоке
client.loop_start()

# Запуск обновления GUI
update_gui()

# Запуск главного цикла tkinter
root.mainloop()

# Остановка MQTT клиента при закрытии окна
client.loop_stop()
client.disconnect()