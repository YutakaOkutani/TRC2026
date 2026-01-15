from gpiozero import LED
from time import sleep
import signal
import sys

# ピンの設定 (BCM番号で指定)
led_red = LED(5)
led_green = LED(6)

def safe_exit(signum, frame):
    """強制終了時(Ctrl+C)にLEDを消灯して終了する"""
    print("\nテストを終了します。すべてのLEDを消灯します。")
    led_red.off()
    led_green.off()
    sys.exit(0)

# Ctrl+C を検知して安全に終了する設定
signal.signal(signal.SIGINT, safe_exit)

print("--- LED Test Start ---")
print("GPIO 5 (Pin 29) and GPIO 6 (Pin 31)")
print("Ctrl+C で終了します\n")

try:
    while True:
        # パターン1: GPIO 5のみ点灯
        print("GPIO 5 : ON  | GPIO 6 : OFF")
        led_red.on()
        led_green.off()
        sleep(1)

        # パターン2: GPIO 6のみ点灯
        print("GPIO 5 : OFF | GPIO 6 : ON")
        led_red.off()
        led_green.on()
        sleep(1)

        # パターン3: 両方点灯
        print("GPIO 5 : ON  | GPIO 6 : ON")
        led_red.on()
        led_green.on()
        sleep(1)

        # パターン4: 両方消灯
        print("GPIO 5 : OFF | GPIO 6 : OFF")
        led_red.off()
        led_green.off()
        sleep(1)

except Exception as e:
    print(f"エラーが発生しました: {e}")
    led_red.off()
    led_green.off()