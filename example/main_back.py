import os
import re
import logging
import threading
import time
from flask import Flask, jsonify, request

app = Flask(__name__)

# Настройка логирования
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)

# Путь к файлу main.py
MAIN_PY_PATH = r"D:\Projects\esp\micropython\test_project\main.py"

# Глобальные переменные для хранения информации о прошивке
LATEST_VERSION = "0.0.0"
UPDATE_FILES = {}
LAST_MODIFIED_TIME = 0


def read_file_content(file_path):
    if os.path.exists(file_path):
        try:
            with open(file_path, 'r', encoding='utf-8') as file:
                return file.read()
        except UnicodeDecodeError:
            encodings = ['cp1251', 'iso-8859-1', 'windows-1252']
            for enc in encodings:
                try:
                    with open(file_path, 'r', encoding=enc) as file:
                        return file.read()
                except UnicodeDecodeError:
                    continue
            return "Unable to read file due to encoding issues"
    return "File not found"


def extract_version(content):
    version_pattern = r'CURRENT_VERSION\s*=\s*"(\d{1,3}\.\d{1,3}\.\d{1,3})"'
    match = re.search(version_pattern, content)
    if match:
        version = match.group(1)
        if all(0 <= int(num) <= 255 for num in version.split('.')):
            return version
    return "0.0.0"


def update_firmware_info():
    global LATEST_VERSION, UPDATE_FILES, LAST_MODIFIED_TIME

    current_modified_time = os.path.getmtime(MAIN_PY_PATH)
    if current_modified_time != LAST_MODIFIED_TIME:
        logger.info("Обнаружены изменения в файле прошивки. Обновление информации...")
        LAST_MODIFIED_TIME = current_modified_time

        file_content = read_file_content(MAIN_PY_PATH)
        LATEST_VERSION = extract_version(file_content)
        UPDATE_FILES = {'main.py': file_content}

        logger.info(f"Информация о прошивке обновлена. Новая версия: {LATEST_VERSION}")


def check_for_updates():
    while True:
        update_firmware_info()
        time.sleep(60)  # Проверка каждую минуту


# Запускаем поток для периодической проверки обновлений
update_thread = threading.Thread(target=check_for_updates, daemon=True)
update_thread.start()


@app.route('/check_update')
def check_update():
    current_version = request.args.get('version', '0.0.0')
    device_ip = request.remote_addr
    logger.info(f"Получен запрос на проверку обновления от устройства {device_ip}. Текущая версия: {current_version}")

    update_available = current_version != LATEST_VERSION
    if update_available:
        logger.info(f"Для устройства {device_ip} доступно обновление с версии {current_version} до {LATEST_VERSION}")
    else:
        logger.info(f"Для устройства {device_ip} обновление не требуется. Текущая версия: {current_version}")

    return jsonify({
        'update_available': update_available,
        'latest_version': LATEST_VERSION
    })


@app.route('/update')
def get_update():
    device_ip = request.remote_addr
    logger.info(f"Начало обновления для устройства {device_ip}")
    logger.info(f"Отправка файлов обновления устройству {device_ip}")
    return jsonify(UPDATE_FILES)


@app.route('/update_complete')
def update_complete():
    device_ip = request.remote_addr
    new_version = request.args.get('version', 'неизвестно')
    logger.info(f"Устройство {device_ip} сообщает о завершении обновления. Новая версия: {new_version}")
    return "OK"


@app.route('/ping')
def ping():
    device_ip = request.remote_addr
    logger.info(f"Получен пинг от устройства {device_ip}")
    return "pong"


if __name__ == '__main__':
    logger.info(f"Сервер запущен в режиме отладки. Текущая версия прошивки: {LATEST_VERSION}")
    app.run(host='0.0.0.0', port=5000, debug=True)