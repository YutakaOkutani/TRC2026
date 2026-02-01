import signal
import sys
import time

from gpiozero import LED
from gpiozero.pins.lgpio import LGPIOFactory

# --- main.py と同じ GPIO 定義／点灯インターバル ---
PIN_LED_RED = 5   # GPIO 5  (Pin 29)
PIN_LED_GREEN = 6 # GPIO 6  (Pin 31)

LED_INTERVAL_PHASE0 = 5
LED_INTERVAL_PHASE2 = 3
LED_INTERVAL_PHASE3 = 10
LED_INTERVAL_PHASE3_NEAR = 2
LED_INTERVAL_PHASE5 = 2

# main.py の loop 相当の周期 (0.1s) でカウンタを進める
LOOP_DT = 0.1

pin_factory = LGPIOFactory()
led_red = LED(PIN_LED_RED, pin_factory=pin_factory)
led_green = LED(PIN_LED_GREEN, pin_factory=pin_factory)
led_blink_timer = 0


def toggle_led(led, timer, interval):
    """main.py と同じトグル処理。interval はカウンタ値で管理。"""
    if led is None:
        return
    if (timer // interval) % 2 == 0:
        led.on()
    else:
        led.off()


def signal_led(times=3):
    """起動確認: 赤＋緑を times 回点滅。"""
    for _ in range(times):
        led_red.on()
        led_green.on()
        time.sleep(0.2)
        led_red.off()
        led_green.off()
        time.sleep(0.2)


def advance():
    """グローバルタイマを進めて返す。"""
    global led_blink_timer
    led_blink_timer += 1
    return led_blink_timer


def phase0_falling(duration=6):
    """PH0: 落下中 - 赤ゆっくり点滅 / 緑消灯"""
    end = time.time() + duration
    print("[PH0] Falling: red blink interval=5, green off")
    while time.time() < end:
        toggle_led(led_red, advance(), LED_INTERVAL_PHASE0)
        led_green.off()
        time.sleep(LOOP_DT)


def phase1_parachute(duration=4):
    """PH1: パラ分離 - 赤点灯 / 緑消灯 (固定)"""
    print("[PH1] Parachute separation: red ON, green OFF")
    led_red.on()
    led_green.off()
    time.sleep(duration)


def phase2_calibration(duration=6):
    """PH2: 姿勢キャリブレーション - 赤点滅 / 緑点灯"""
    end = time.time() + duration
    print("[PH2] BNO calibration: red blink interval=3, green ON")
    led_green.on()
    while time.time() < end:
        toggle_led(led_red, advance(), LED_INTERVAL_PHASE2)
        time.sleep(LOOP_DT)
    led_red.off()


def phase3_gps_search(duration=5):
    """PH3 (GPS検索中): 赤消灯 / 緑ゆっくり点滅"""
    end = time.time() + duration
    print("[PH3] GPS search (no fix): green blink interval=10, red OFF")
    led_red.off()
    while time.time() < end:
        toggle_led(led_green, advance(), LED_INTERVAL_PHASE3)
        time.sleep(LOOP_DT)
    led_green.off()


def phase3_gps_close(duration=5):
    """PH3 (目的地付近): 赤消灯 / 緑速め点滅"""
    end = time.time() + duration
    print("[PH3] GPS close to target: green blink interval=2, red OFF")
    led_red.off()
    while time.time() < end:
        toggle_led(led_green, advance(), LED_INTERVAL_PHASE3_NEAR)
        time.sleep(LOOP_DT)
    led_green.off()


def phase4_camera_search(duration=6):
    """PH4: カメラサーチ - 赤消灯 / 緑点灯"""
    print("[PH4] Camera searching: green ON, red OFF")
    led_red.off()
    led_green.on()
    time.sleep(duration)
    led_green.off()


def phase5_approach(duration=8):
    """PH5: 接近 - 赤と緑を交互点滅 (間隔2)"""
    end = time.time() + duration
    print("[PH5] Approaching cone: alternate red/green every 2 counts")
    while time.time() < end:
        if (advance() // LED_INTERVAL_PHASE5) % 2 == 0:
            led_red.on()
            led_green.off()
        else:
            led_red.off()
            led_green.on()
        time.sleep(LOOP_DT)
    led_red.off()
    led_green.off()


def phase6_goal(duration=4):
    """PH6: ゴール - 両方点灯"""
    print("[PH6] Goal: red ON, green ON")
    led_red.on()
    led_green.on()
    time.sleep(duration)


def all_off():
    """安全のため全消灯。"""
    led_red.off()
    led_green.off()


def safe_exit(signum, frame):
    """Ctrl+C 時に安全に消灯して終了。"""
    print("\nテストを終了します。すべてのLEDを消灯します。")
    all_off()
    sys.exit(0)


def run_demo():
    """main.py の全フェーズ分の LED パターンを順番に再生する。"""
    print("--- LED Test Start (aligned with main.py phases) ---")
    print("Ctrl+C でいつでも終了できます\n")
    signal_led(3)

    sequence = [
        phase0_falling,
        phase1_parachute,
        phase2_calibration,
        phase3_gps_search,
        phase3_gps_close,
        phase4_camera_search,
        phase5_approach,
        phase6_goal,
    ]

    for func in sequence:
        func()
        time.sleep(0.5)

    print("全フェーズのパターンを再生しました。LEDを消灯して終了します。")
    all_off()


if __name__ == "__main__":
    signal.signal(signal.SIGINT, safe_exit)
    try:
        run_demo()
    except Exception as e:
        print(f"エラーが発生しました: {e}")
        all_off()
