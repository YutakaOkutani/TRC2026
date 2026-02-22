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
    TIMEOUT_PHASE_0,
)
from cansat_mission.phases.base import BasePhaseHandler


class Phase0Handler(BasePhaseHandler):
    def execute(self, controller, snapshot):
        led_red = controller.devices.get(DEVICE_LED_RED)
        led_green = controller.devices.get(DEVICE_LED_GREEN)

        entry_marker = getattr(controller, "phase_entry_time", None)
        if getattr(controller, "phase0_entry_marker", None) != entry_marker:
            controller.phase0_entry_marker = entry_marker
            controller.phase0_initial_alt = snapshot["alt"]
            print("phase0 : falling")
            print(f"Start Altitude: {controller.phase0_initial_alt:.2f}m")

        controller.toggle_led(led_red, controller.led_blink_timer, interval=LED_INTERVAL_PHASE0)
        if led_green:
            led_green.off()

        initial_alt = controller.phase0_initial_alt if controller.phase0_initial_alt is not None else snapshot["alt"]
        is_impact = snapshot["fall"] > IMPACT_FALL_THRESHOLD
        altitude_diff = initial_alt - snapshot["alt"]
        is_drop = altitude_diff > DROP_ALTITUDE_DIFF_THRESHOLD

        if is_drop:
            print(f"Detected Drop: {altitude_diff:.2f}m")
            controller.state.update_navigation(phase=int(Phase.PHASE1))
            controller.time_phase1_start = time.time()
            return

        if is_impact:
            print(f"Detected Impact: {snapshot['fall']:.2f}m/s^2")
            controller.state.update_navigation(phase=int(Phase.PHASE1))
            controller.time_phase1_start = time.time()
            return

        if controller.time_phase1_start is None:
            phase0_start = entry_marker if entry_marker is not None else time.time()
            if time.time() - phase0_start > TIMEOUT_PHASE_0:
                print("Phase0 TIMEOUT: Force proceed (Sensor failure?)")
                controller.state.update_navigation(phase=int(Phase.PHASE1))
                controller.time_phase1_start = time.time()


def _print_direct_run_help():
    print("phase0.py is a phase handler module and does not run mission logic by itself.")
    print("Run mission from project root with: python3 main.py")


if __name__ == "__main__":
    _print_direct_run_help()
