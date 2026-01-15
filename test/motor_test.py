import time

from gpiozero import PWMOutputDevice, DigitalOutputDevice
from gpiozero.pins.lgpio import LGPIOFactory

# --- GPIO Pin Definition ---
# DRV8256E: Phase/Enable Mode
# EN (Enable) -> PWM (Speed)
# PH (Phase)  -> High/Low (Direction)

# Motor 1 (Right or Left)
PIN_EN1 = 12  # Enable (PWM Speed)
PIN_PH1 = 13  # Phase (Direction)

# Motor 2 (Left or Right)
PIN_EN2 = 19   # Enable (PWM Speed)
PIN_PH2 = 17  # Phase (Direction)

# PWM Parameters
PWM_FREQ = 1000  # PWM Frequency in Hz

# gpiozero devices (created in setup())
pin_factory = None
motor_1_pwm = None
motor_1_dir = None
motor_2_pwm = None
motor_2_dir = None
motor_state = {
    'A': {'speed': 0.0, 'direction': 1},
    'B': {'speed': 0.0, 'direction': 1},
}


def setup():
    """Initialize gpiozero devices."""
    global pin_factory, motor_1_pwm, motor_1_dir, motor_2_pwm, motor_2_dir

    pin_factory = LGPIOFactory()
    motor_1_pwm = PWMOutputDevice(PIN_EN1, pin_factory=pin_factory, frequency=PWM_FREQ, initial_value=0)
    motor_1_dir = DigitalOutputDevice(PIN_PH1, pin_factory=pin_factory, initial_value=False)
    motor_2_pwm = PWMOutputDevice(PIN_EN2, pin_factory=pin_factory, frequency=PWM_FREQ, initial_value=0)
    motor_2_dir = DigitalOutputDevice(PIN_PH2, pin_factory=pin_factory, initial_value=False)

    stop()

    print("Setup Complete: gpiozero initialized.")


def _ramp_pwm(pwm_dev, start_speed, target_speed, ramp_time, step_interval=0.05):
    """Ramp PWM duty in small steps to avoid sudden current draw."""
    if pwm_dev is None:
        return target_speed

    # Immediate set if ramping is disabled or step is invalid.
    if ramp_time <= 0 or step_interval <= 0:
        pwm_dev.value = max(0.0, min(1.0, target_speed / 100.0))
        return target_speed

    steps = max(1, int(ramp_time / step_interval))
    step_duration = ramp_time / steps
    for step in range(1, steps + 1):
        duty = start_speed + (target_speed - start_speed) * (step / steps)
        pwm_dev.value = max(0.0, min(1.0, duty / 100.0))
        time.sleep(step_duration)

    return target_speed


def set_motor(motor_side, speed, direction, ramp_time=0.6, step_interval=0.05):
    """
    Control motor duty and direction with a soft-start ramp.
    :param motor_side: 'A' (Right) or 'B' (Left)
    :param speed: PWM Duty Cycle (0 - 100)
    :param direction: 1 (Forward/High) or 0 (Reverse/Low)
    :param ramp_time: Time in seconds to ramp between duty changes.
    :param step_interval: Interval between duty steps.
    """
    if motor_side == 'A':
        pwm_dev = motor_1_pwm
        dir_dev = motor_1_dir
    elif motor_side == 'B':
        pwm_dev = motor_2_pwm
        dir_dev = motor_2_dir
    else:
        return

    if pwm_dev is None or dir_dev is None:
        return

    state = motor_state[motor_side]
    current_speed = state['speed']
    current_direction = state['direction']

    # If the direction changes, ramp to zero first to reduce stress on the driver.
    if current_speed > 0 and direction != current_direction:
        current_speed = _ramp_pwm(pwm_dev, current_speed, 0, ramp_time / 2, step_interval)

    # Set direction (PH Pin)
    # Hardware wiring uses inverted logic: logical "forward" (1) needs PH low.
    dir_dev.value = 0 if direction == 1 else 1

    # Set PWM duty with ramp (EN Pin - PWM 0.0-1.0)
    target_speed = max(0.0, min(100.0, speed))
    current_speed = _ramp_pwm(pwm_dev, current_speed, target_speed, ramp_time, step_interval)
    state['speed'] = current_speed
    state['direction'] = direction


def stop():
    """Stop both motors and reset cached speed state."""
    if motor_1_pwm:
        motor_1_pwm.value = 0
    if motor_2_pwm:
        motor_2_pwm.value = 0
    if motor_1_dir:
        motor_1_dir.off()
    if motor_2_dir:
        motor_2_dir.off()
    motor_state['A']['speed'] = 0.0
    motor_state['B']['speed'] = 0.0


def main():
    try:
        setup()
        print("Motor Test Start: gpiozero with DRV8256E")

        while True:
            # --- 1. Forward ---
            print("Forward: 50%")
            set_motor('A', 50, 1)  # Right Forward
            set_motor('B', 50, 1)  # Left Forward
            time.sleep(2)

            # --- 2. Stop ---
            print("Stop")
            stop()
            time.sleep(1)

            # --- 3. Reverse ---
            print("Reverse: 50%")
            set_motor('A', 50, 0)  # Right Reverse
            set_motor('B', 50, 0)  # Left Reverse
            time.sleep(2)

            # --- 4. Turn ---
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
