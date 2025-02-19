def connect_wifi(ssid, password, max_attempts=5, retry_delay=5):
    set_color(255, 0, 0, 3)
    wlan = network.WLAN(network.STA_IF)
    wlan.active(True)

    if wlan.isconnected():
        print("Already connected to WiFi")
        set_color(0, 255, 0, 3)
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
            # print("Wi-Fi connected. Network config:", wlan.ifconfig())
            print(f'WiFi connected, IP addr is {wlan.ifconfig()[0]}')
            set_color(0, 255, 0, 3)
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