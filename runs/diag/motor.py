import sys
import time
from pathlib import Path

# Allow running this file directly (e.g. `python runs/motor_diag.py`) by adding
# the repository root to sys.path so `csmn` can be imported.
REPO_ROOT = Path(__file__).resolve().parents[2]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from gpiozero import PWMOutputDevice, DigitalOutputDevice
from gpiozero.pins.lgpio import LGPIOFactory

from csmn.const import (
    MANUAL_TURN_SPEED_RATIO,
    MOTOR_DIR_INVERT_1,
    MOTOR_DIR_INVERT_2,
    MOTOR_SPEED_OFFSET_1,
    MOTOR_SPEED_OFFSET_2,
    MOTOR_SPEED_SCALE_1,
    MOTOR_SPEED_SCALE_2,
)
from csmn.mgr.mtr_mgr import get_manual_drive_pattern

# --- GPIO Pin Definition ---
# DRV8256E: Phase/Enable Mode
# EN (Enable) -> PWM (Speed)
# PH (Phase)  -> High/Low (Direction)

# Motor 1 (Left)
PIN_EN1 = 12  # Enable (PWM Speed)
PIN_PH1 = 13  # Phase (Direction)

# Motor 2 (Right)
PIN_EN2 = 19   # Enable (PWM Speed)
PIN_PH2 = 17  # Phase (Direction)

# PWM Parameters
PWM_FREQ = 1000  # PWM Frequency in Hz
DEFAULT_SPEED = 100  # Default duty for manual control (0-100)
SPEED_STEP = 5  # Duty adjustment step for interactive test (0-100)
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

    print(
        f"Setup Complete: gpiozero initialized. "
        f"speed_scale A={MOTOR_SPEED_SCALE_1:.3f}, B={MOTOR_SPEED_SCALE_2:.3f}, "
        f"speed_offset A={MOTOR_SPEED_OFFSET_1:.1f}, B={MOTOR_SPEED_OFFSET_2:.1f}"
    )


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


def _ramp_pwm_dual(pwm_a, start_a, target_a, pwm_b, start_b, target_b, ramp_time, step_interval=0.05):
    """Ramp two PWM devices together so both motors start/stop in sync."""
    if pwm_a is None and pwm_b is None:
        return start_a, start_b

    # Immediate set if ramping is disabled or step is invalid.
    if ramp_time <= 0 or step_interval <= 0:
        if pwm_a is not None:
            pwm_a.value = max(0.0, min(1.0, target_a / 100.0))
        if pwm_b is not None:
            pwm_b.value = max(0.0, min(1.0, target_b / 100.0))
        return target_a, target_b

    steps = max(1, int(ramp_time / step_interval))
    step_duration = ramp_time / steps
    for step in range(1, steps + 1):
        duty_a = start_a + (target_a - start_a) * (step / steps)
        duty_b = start_b + (target_b - start_b) * (step / steps)
        if pwm_a is not None:
            pwm_a.value = max(0.0, min(1.0, duty_a / 100.0))
        if pwm_b is not None:
            pwm_b.value = max(0.0, min(1.0, duty_b / 100.0))
        time.sleep(step_duration)

    return target_a, target_b


def _apply_speed_scale(speed, motor_side):
    """Apply per-motor PWM trim (scale + offset) to compensate differences."""
    scale = MOTOR_SPEED_SCALE_1 if motor_side == 'A' else MOTOR_SPEED_SCALE_2
    offset = MOTOR_SPEED_OFFSET_1 if motor_side == 'A' else MOTOR_SPEED_OFFSET_2
    adjusted = float(speed) * max(0.0, float(scale)) + float(offset)
    return max(0.0, min(100.0, adjusted))


def set_motor(motor_side, speed, direction, ramp_time=0.6, step_interval=0.05):
    """
    Control motor duty and direction with a soft-start ramp.
    :param motor_side: 'A' (Left / MTR1) or 'B' (Right / MTR2)
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
    # Match production polarity handling through invert flags.
    invert = MOTOR_DIR_INVERT_1 if motor_side == 'A' else MOTOR_DIR_INVERT_2
    dir_dev.value = 1 if (bool(direction) ^ bool(invert)) else 0

    # Set PWM duty with ramp (EN Pin - PWM 0.0-1.0)
    target_speed = _apply_speed_scale(speed, motor_side)
    current_speed = _ramp_pwm(pwm_dev, current_speed, target_speed, ramp_time, step_interval)
    state['speed'] = current_speed
    state['direction'] = direction


def set_motors(speed_a, dir_a, speed_b, dir_b, ramp_time=0.6, step_interval=0.05):
    """
    Control both motors together with a synchronized ramp.
    :param speed_a: PWM Duty Cycle (0 - 100) for Motor A
    :param dir_a: 1 (Forward/High) or 0 (Reverse/Low) for Motor A
    :param speed_b: PWM Duty Cycle (0 - 100) for Motor B
    :param dir_b: 1 (Forward/High) or 0 (Reverse/Low) for Motor B
    :param ramp_time: Time in seconds to ramp between duty changes.
    :param step_interval: Interval between duty steps.
    """
    if motor_1_pwm is None or motor_1_dir is None or motor_2_pwm is None or motor_2_dir is None:
        return

    state_a = motor_state['A']
    state_b = motor_state['B']
    current_a = state_a['speed']
    current_b = state_b['speed']

    # If either direction changes, ramp both to zero first to reduce stress and keep sync.
    if (current_a > 0 and dir_a != state_a['direction']) or (current_b > 0 and dir_b != state_b['direction']):
        current_a, current_b = _ramp_pwm_dual(
            motor_1_pwm, current_a, 0,
            motor_2_pwm, current_b, 0,
            ramp_time / 2, step_interval
        )

    # Match production polarity handling through invert flags.
    motor_1_dir.value = 1 if (bool(dir_a) ^ bool(MOTOR_DIR_INVERT_1)) else 0
    motor_2_dir.value = 1 if (bool(dir_b) ^ bool(MOTOR_DIR_INVERT_2)) else 0

    target_a = _apply_speed_scale(speed_a, 'A')
    target_b = _apply_speed_scale(speed_b, 'B')
    current_a, current_b = _ramp_pwm_dual(
        motor_1_pwm, current_a, target_a,
        motor_2_pwm, current_b, target_b,
        ramp_time, step_interval
    )

    state_a['speed'] = current_a
    state_a['direction'] = dir_a
    state_b['speed'] = current_b
    state_b['direction'] = dir_b


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



def _apply_manual_drive_pattern(cmd, speed=DEFAULT_SPEED):
    """Apply the production manual drive mapping through the local test motor driver."""
    pattern = get_manual_drive_pattern(cmd, speed)
    if pattern is None:
        return False
    set_motors(
        pattern["speed_a"],
        int(pattern["forward_a"]),
        pattern["speed_b"],
        int(pattern["forward_b"]),
    )
    return True


def drive_forward(speed=DEFAULT_SPEED):
    """Drive both motors forward."""
    _apply_manual_drive_pattern("w", speed)


def drive_backward(speed=DEFAULT_SPEED):
    """Drive both motors backward."""
    _apply_manual_drive_pattern("s", speed)


def turn_left(speed=DEFAULT_SPEED):
    """Steer left with differential forward speeds (no reverse)."""
    fast = max(0.0, min(100.0, float(speed)))
    slow = max(0.0, min(100.0, fast * MANUAL_TURN_SPEED_RATIO))
    # A: MTR1, B: MTR2
    set_motors(slow, 1, fast, 1)


def turn_right(speed=DEFAULT_SPEED):
    """Steer right with differential forward speeds (no reverse)."""
    fast = max(0.0, min(100.0, float(speed)))
    slow = max(0.0, min(100.0, fast * MANUAL_TURN_SPEED_RATIO))
    # A: MTR1, B: MTR2
    set_motors(fast, 1, slow, 1)


def _read_key():
    """
    Read a single key press and normalize to movement commands.
    Supports WASD and arrow keys on Windows and POSIX terminals.
    Returns the lowercase command character or '' if unknown.
    """
    try:
        import msvcrt  # Windows single-key capture

        key = msvcrt.getch()
        if key in (b'\x00', b'\xe0'):
            special = msvcrt.getch()
            arrow_map = {
                b'H': 'w',  # Up
                b'P': 's',  # Down
                b'K': 'a',  # Left
                b'M': 'd',  # Right
            }
            return arrow_map.get(special, '')
        return key.decode('utf-8', 'ignore').lower()
    except ImportError:
        import termios
        import tty

        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            ch1 = sys.stdin.read(1)
            if ch1 == '\x1b':  # Escape sequence for arrows
                ch2 = sys.stdin.read(1)
                if ch2 == '[':
                    ch3 = sys.stdin.read(1)
                    arrow_map = {
                        'A': 'w',  # Up
                        'B': 's',  # Down
                        'D': 'a',  # Left
                        'C': 'd',  # Right
                    }
                    return arrow_map.get(ch3, '')
                return ''
            return ch1.lower()
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)


def _get_command():
    """
    Wrapper to fetch a single command key.
    Falls back to input() if raw capture fails.
    """
    try:
        return _read_key()
    except Exception:
        try:
            return input("Enter command: ").strip().lower()[:1]
        except EOFError:
            return ''


def _clamp_speed(speed):
    return max(0.0, min(100.0, float(speed)))


def main():
    current_speed = float(DEFAULT_SPEED)
    try:
        setup()
        print(
            "Motor Control Ready "
            "(W/A/S/D or Arrow Keys, +=faster, -=slower, space=stop, q=quit)"
        )
        print(f"Current Duty: {current_speed:.0f}%")
        while True:
            cmd = _get_command()

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

            if cmd in ('+', '='):
                current_speed = _clamp_speed(current_speed + SPEED_STEP)
                print(f"Duty Up -> {current_speed:.0f}%")
                continue

            if cmd in ('-', '_'):
                current_speed = _clamp_speed(current_speed - SPEED_STEP)
                print(f"Duty Down -> {current_speed:.0f}%")
                continue

            # Apply a short buffer before acting to reduce regen stress.
            time.sleep(COMMAND_BUFFER_SEC)

            if cmd == 'w':
                print(f"Forward ({current_speed:.0f}%)")
                drive_forward(current_speed)
            elif cmd == 's':
                print(f"Backward ({current_speed:.0f}%)")
                drive_backward(current_speed)
            elif cmd == 'a':
                print(f"Left ({current_speed:.0f}%)")
                turn_left(current_speed)
            elif cmd == 'd':
                print(f"Right ({current_speed:.0f}%)")
                turn_right(current_speed)
            else:
                print(f"Unknown command '{cmd}'. Use W/A/S/D, space, or q.")

    except KeyboardInterrupt:
        print("\nExiting...")
        stop()


if __name__ == "__main__":
    main()
