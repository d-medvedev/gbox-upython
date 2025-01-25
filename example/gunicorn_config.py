import multiprocessing

bind = "0.0.0.0:5000"
workers = multiprocessing.cpu_count() * 2 + 1
worker_class = 'sync'
worker_connections = 1000
timeout = 30
max_requests = 2000
daemon = False
accesslog = "-"
errorlog = "-"
loglevel = "info"

def on_starting(server):
    from app import update_thread
    update_thread.start()

