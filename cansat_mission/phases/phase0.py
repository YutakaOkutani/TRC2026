import os
import sys
import time

if __package__ is None or __package__ == "":
    sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..")))

from cansat_mission.constants import (
    DEVICE_LED_GREEN,
    DEVICE_LED_RED,
    DROP_ALTITUDE_DIFF_THRESHOLD,
    IMPACT_FALL_THRESHOLD,
    LED_INTERVAL_PHASE0,
    Phase,
    SHORT_SLEEP,
    TIMEOUT_PHASE_0,
)
from cansat_mission.phases.base import BasePhaseHandler


class Phase0Handler(BasePhaseHandler):
    def execute(self, controller, snapshot):
        led_red = controller.devices.get(DEVICE_LED_RED)
        led_green = controller.devices.get(DEVICE_LED_GREEN)
        print("phase0 : falling")
        controller.toggle_led(led_red, controller.led_blink_timer, interval=LED_INTERVAL_PHASE0)
        if led_green:
            led_green.off()
        start_time = time.time()
        initial_alt = snapshot["alt"]
        print(f"Start Altitude: {initial_alt:.2f}m")
        while True:
            controller.led_blink_timer += 1
            controller.toggle_led(led_red, controller.led_blink_timer, interval=LED_INTERVAL_PHASE0)
            current_snapshot = controller.state.snapshot()
            is_impact = current_snapshot["fall"] > IMPACT_FALL_THRESHOLD
            altitude_diff = initial_alt - current_snapshot["alt"]
            is_drop = altitude_diff > DROP_ALTITUDE_DIFF_THRESHOLD
            if is_drop:
                print(f"Detected Drop: {altitude_diff:.2f}m")
                break
            if is_impact:
                print(f"Detected Impact: {current_snapshot['fall']:.2f}m/s^2")
                break
            if time.time() - start_time > TIMEOUT_PHASE_0:
                print("Phase0 TIMEOUT: Force proceed (Sensor failure?)")
                break
            time.sleep(SHORT_SLEEP)
        controller.state.update_navigation(phase=int(Phase.PHASE1))
        controller.time_phase1_start = time.time()


def _print_direct_run_help():
    print("phase0.py is a phase handler module and does not run mission logic by itself.")
    print("Run mission from project root with: python3 main.py")


if __name__ == "__main__":
    _print_direct_run_help()
