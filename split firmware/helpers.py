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