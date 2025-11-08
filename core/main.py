
# hello_app.py
import time
def run():
    i = 0
    while True:
        print("Hello from BLE app:", i)
        i += 1
        time.sleep(1)
if __name__ == "__main__":
    run()
