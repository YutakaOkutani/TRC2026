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
DEFAULT_SPEED = 50  # Default duty for manual control (0-100)
COMMAND_BUFFER_SEC = 0.25  # Delay before applying a new command to avoid regen spikes

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


def drive_forward(speed=DEFAULT_SPEED):
    """Drive both motors forward."""
    set_motor('A', speed, 1)
    set_motor('B', speed, 1)


def drive_backward(speed=DEFAULT_SPEED):
    """Drive both motors backward."""
    set_motor('A', speed, 0)
    set_motor('B', speed, 0)


def turn_left(speed=DEFAULT_SPEED):
    """Pivot left: right motor forward, left motor reverse."""
    set_motor('A', speed, 1)
    set_motor('B', speed, 0)


def turn_right(speed=DEFAULT_SPEED):
    """Pivot right: right motor reverse, left motor forward."""
    set_motor('A', speed, 0)
    set_motor('B', speed, 1)


def main():
    try:
        setup()
        print("Motor Control Ready (W/A/S/D, space=stop, q=quit)")
        while True:
            cmd = input("Enter command: ").strip().lower()

            # Empty input -> ignore to avoid jitter.
            if not cmd:
                continue

            # Take only the first character for simplicity.
            cmd = cmd[0]

            if cmd == 'q':
                print("Quit requested.")
                break

            if cmd == ' ':
                print("Stop")
                stop()
                continue

            # Apply a short buffer before acting to reduce regen stress.
            time.sleep(COMMAND_BUFFER_SEC)

            if cmd == 'w':
                print("Forward")
                drive_forward()
            elif cmd == 's':
                print("Backward")
                drive_backward()
            elif cmd == 'a':
                print("Left")
                turn_left()
            elif cmd == 'd':
                print("Right")
                turn_right()
            else:
                print(f"Unknown command '{cmd}'. Use W/A/S/D, space, or q.")

    except KeyboardInterrupt:
        print("\nExiting...")
        stop()


if __name__ == "__main__":
    main()
