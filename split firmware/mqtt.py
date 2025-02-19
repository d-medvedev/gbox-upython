def on_message(topic, msg):
    print(topic, msg)
    if topic == (system['main']['base_topic'] + system['mqtt_topics_sub']['sgs_topic']).encode():
        try:
            msg_str = json_replace_errors(msg)

            try:
                new_config = json.loads(msg_str)
            except ValueError as e:
                print(f"Error: Invalid value in JSON - {e}")
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
                    payload = json.dumps(current_config)
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

    if topic == (system['main']['base_topic'] + system['mqtt_topics_sub']['cmd_topic']).encode():
        try:
            msg_str = json_replace_errors(msg)

            try:
                command = json.loads(msg_str)
            except ValueError as e:
                print(f"Error: Invalid value in JSON - {e}")
                return

            try:
                if 'lamp_pwm' in command:
                    lamp_value = max(0, min(100, command['lamp_pwm']))
                    light_pwm_pin.duty(int(1024 * lamp_value / 100))
                    set_color(0,0,lamp_value,4)
                if 'vent_pwm' in command:
                    vent_value = max(0, min(100, command['vent_pwm']))
                    vent_pwm_pin.duty(int(1024 * vent_value / 100))
                    set_color(0,0,vent_value,1)
                if 'pump' in command:
                    if command['pump'] == 'on':
                        pump_pin.on()
                        set_color(0, 0, 100, 5)
                    else:
                        pump_pin.off()
                        set_color(0, 0, 0, 5)
            except Exception as e:
                print(f"Error processing packet: {e}")

        except Exception as e:
            print(f"Error processing message: {e}")

def send_to_mqtt(topic, payload):
    with queue_lock:
        if len(mqtt_queue) < mqtt_queue.maxlen:
            mqtt_queue.append({'topic': topic, 'payload': payload})
            return True
        return False