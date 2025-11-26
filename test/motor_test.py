import time

from gpiozero import PWMOutputDevice, DigitalOutputDevice
from gpiozero.pins.lgpio import LGPIOFactory

# --- GPIO Pin Definition ---
# DRV8256E: Phase/Enable Mode
# EN (Enable) -> PWM (Speed)
# PH (Phase)  -> High/Low (Direction)

# Motor A (Right or Left)
PIN_ENA = 17  # Enable (PWM Speed)
PIN_PHA = 19  # Phase (Direction)

# Motor B (Left or Right)
PIN_ENB = 2   # Enable (PWM Speed)
PIN_PHB = 13  # Phase (Direction)

# PWM Parameters
PWM_FREQ = 20000

# gpiozero devices (setup()で初期化)
pin_factory = None
motor_a_pwm = None
motor_a_dir = None
motor_b_pwm = None
motor_b_dir = None


def setup():
    """GPIOの初期設定"""
    global pin_factory, motor_a_pwm, motor_a_dir, motor_b_pwm, motor_b_dir

    pin_factory = LGPIOFactory()
    motor_a_pwm = PWMOutputDevice(PIN_ENA, pin_factory=pin_factory, frequency=PWM_FREQ, initial_value=0)
    motor_a_dir = DigitalOutputDevice(PIN_PHA, pin_factory=pin_factory, initial_value=False)
    motor_b_pwm = PWMOutputDevice(PIN_ENB, pin_factory=pin_factory, frequency=PWM_FREQ, initial_value=0)
    motor_b_dir = DigitalOutputDevice(PIN_PHB, pin_factory=pin_factory, initial_value=False)

    # 初期化（安全のため停止）
    stop()

    print("Setup Complete: gpiozero initialized.")


def set_motor(motor_side, speed, direction):
    """
    モーターを制御する関数
    :param motor_side: 'A' (Right) or 'B' (Left)
    :param speed: PWM Duty Cycle (0 - 100)
    :param direction: 1 (Forward/High) or 0 (Reverse/Low)
    """
    if motor_side == 'A':
        pwm_dev = motor_a_pwm
        dir_dev = motor_a_dir
    elif motor_side == 'B':
        pwm_dev = motor_b_pwm
        dir_dev = motor_b_dir
    else:
        return

    if pwm_dev is None or dir_dev is None:
        return

    # 方向制御 (PH Pin)
    dir_dev.value = 1 if direction == 1 else 0

    # 速度制御 (EN Pin - PWM 0.0-1.0)
    pwm_dev.value = max(0.0, min(1.0, speed / 100.0))


def stop():
    """全モーターを停止（Coast）"""
    if motor_a_pwm:
        motor_a_pwm.value = 0
    if motor_b_pwm:
        motor_b_pwm.value = 0
    if motor_a_dir:
        motor_a_dir.off()
    if motor_b_dir:
        motor_b_dir.off()


def main():
    try:
        setup()
        print("Motor Test Start: gpiozero with DRV8256E")

        while True:
            # --- 1. Forward (正転) ---
            print("Forward: 50%")
            set_motor('A', 50, 1)  # Right Forward
            set_motor('B', 50, 1)  # Left Forward
            time.sleep(2)

            # --- 2. Stop (停止) ---
            print("Stop")
            stop()
            time.sleep(1)

            # --- 3. Reverse (逆転) ---
            print("Reverse: 50%")
            set_motor('A', 50, 0)  # Right Reverse
            set_motor('B', 50, 0)  # Left Reverse
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


if __name__ == "__main__":
    main()
