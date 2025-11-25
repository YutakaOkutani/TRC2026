import pigpio
import time

# --- GPIO Pin Definition ---
# DRV8256E: Phase/Enable Mode
# EN (Enable) -> PWM (Speed)
# PH (Phase)  -> High/Low (Direction)

# Motor A (Right or Left)
PIN_ENA = 17  # Enable (PWM Speed)
PIN_PHA = 19   # Phase (Direction)

# Motor B (Left or Right)
PIN_ENB = 2  # Enable (PWM Speed)
PIN_PHB = 13   # Phase (Direction)

# PWM Parameters
PWM_FREQ = 20000  

# --- Initialize pigpio ---
pi = pigpio.pi()

if not pi.connected:
    print("Error: Could not connect to pigpio daemon.")
    print("Please run 'sudo pigpiod' in terminal first.")
    exit()

def setup():
    """GPIOの初期設定"""
    # モード設定 (すべて出力)
    pi.set_mode(PIN_ENA, pigpio.OUTPUT)
    pi.set_mode(PIN_PHA, pigpio.OUTPUT)
    pi.set_mode(PIN_ENB, pigpio.OUTPUT)
    pi.set_mode(PIN_PHB, pigpio.OUTPUT)
    
    # 初期化（安全のため停止）
    stop()
    
    print("Setup Complete: pigpio initialized.")

def set_motor(motor_side, speed, direction):
    """
    モーターを制御する関数
    :param motor_side: 'A' (Right) or 'B' (Left)
    :param speed: PWM Duty Cycle (0 - 100)
    :param direction: 1 (Forward/High) or 0 (Reverse/Low)
    """
    # ピンの選択
    if motor_side == 'A':
        pin_en = PIN_ENA
        pin_ph = PIN_PHA
    elif motor_side == 'B':
        pin_en = PIN_ENB
        pin_ph = PIN_PHB
    else:
        return

    # 方向制御 (PH Pin)
    if direction == 1:
        pi.write(pin_ph, 1) # Forward (例: High)
    else:
        pi.write(pin_ph, 0) # Reverse (例: Low)

    # 速度制御 (EN Pin - PWM)
    # pigpioのset_PWM_dutycycleは 0-255 の値を引数に取る
    # 入力の 0-100 を 0-255 に変換
    duty_value = int((speed / 100.0) * 255)
    
    # 周波数の設定
    pi.set_PWM_frequency(pin_en, PWM_FREQ)
    
    # Duty比の設定
    pi.set_PWM_dutycycle(pin_en, duty_value)

def stop():
    """全モーターを停止（Coast）"""
    pi.set_PWM_dutycycle(PIN_ENA, 0)
    pi.set_PWM_dutycycle(PIN_ENB, 0)

    pi.write(PIN_PHA, 0)
    pi.write(PIN_PHB, 0)

def main():
    try:
        setup()
        print("Motor Test Start: pigpio with DRV8256E")

        while True:
            # --- 1. Forward (正転) ---
            print("Forward: 50%")
            set_motor('A', 50, 1) # Right Forward
            set_motor('B', 50, 1) # Left Forward
            time.sleep(2)

            # --- 2. Stop (停止) ---
            print("Stop")
            stop()
            time.sleep(1)

            # --- 3. Reverse (逆転) ---
            print("Reverse: 50%")
            set_motor('A', 50, 0) # Right Reverse
            set_motor('B', 50, 0) # Left Reverse
            time.sleep(2)

            # --- 4. Turn (旋回テスト) ---
            print("Turn Right: A=Forward, B=Reverse")
            set_motor('A', 40, 1)
            set_motor('B', 40, 0)
            time.sleep(2)

            # --- 5. Stop ---
            print("Stop")
            stop()
            time.sleep(1)

    except KeyboardInterrupt:
        print("\nExiting...")
        stop()
        # pigpioのリソース解放
        pi.stop()

if __name__ == "__main__":
    main()