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
            print("Error response status:", response.status_code)
    except Exception as e:
        print("Error:", str(e))
    return False