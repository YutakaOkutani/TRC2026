import os
import sys
import time

if __package__ is None or __package__ == "":
    sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..")))

from csmn.const import (
    DEVICE_LED_GREEN,
    DEVICE_LED_RED,
    DROP_ALTITUDE_DIFF_THRESHOLD,
    IMPACT_FALL_THRESHOLD,
    LED_INTERVAL_PHASE0,
    PHASE0_DROP_TO_PHASE1_DELAY_SEC,
    PHASE0_SENSOR_STALE_TIMEOUT,
    Phase,
    TIMEOUT_PHASE_0,
)
from csmn.phs.base import BasePhaseHandler


class Phase0Handler(BasePhaseHandler):
    def execute(self, controller, snapshot):
        led_red = controller.devices.get(DEVICE_LED_RED)
        led_green = controller.devices.get(DEVICE_LED_GREEN)

        entry_marker = getattr(controller, "phase_entry_time", None)
        if getattr(controller, "phase0_entry_marker", None) != entry_marker:
            controller.phase0_entry_marker = entry_marker
            controller.phase0_initial_alt = None
            controller.phase0_drop_detect_time = None
            controller.phase0_wait_log_counter = 0
            print("p0 : falling")

        controller.toggle_led(led_red, controller.led_blink_timer, interval=LED_INTERVAL_PHASE0)
        if led_green:
            led_green.off()

        now = time.time()
        bmp_stale_sec = float(getattr(controller, "bmp_stale_sec", TIMEOUT_PHASE_0 + 1.0))
        bno_acc_stale_sec = float(getattr(controller, "bno_acc_stale_sec", TIMEOUT_PHASE_0 + 1.0))
        bmp_valid = (
            getattr(controller, "bmp_last_valid_time", 0.0) > 0.0
            and bmp_stale_sec <= PHASE0_SENSOR_STALE_TIMEOUT
        )
        acc_valid = (
            getattr(controller, "bno_last_acc_time", 0.0) > 0.0
            and bno_acc_stale_sec <= PHASE0_SENSOR_STALE_TIMEOUT
            and snapshot["fall"] is not None
        )

        if bmp_valid and controller.phase0_initial_alt is None:
            controller.phase0_initial_alt = snapshot["alt"]
            print(f"Start Altitude: {controller.phase0_initial_alt:.2f}m")

        initial_alt = controller.phase0_initial_alt
        altitude_diff = 0.0
        if bmp_valid and initial_alt is not None:
            altitude_diff = initial_alt - snapshot["alt"]
        is_drop = bmp_valid and initial_alt is not None and altitude_diff > DROP_ALTITUDE_DIFF_THRESHOLD
        is_impact = acc_valid and snapshot["fall"] > IMPACT_FALL_THRESHOLD

        if not bmp_valid or not acc_valid:
            controller.phase0_wait_log_counter = int(getattr(controller, "phase0_wait_log_counter", 0)) + 1
            if controller.phase0_wait_log_counter % 25 == 0:
                print(
                    "Phase0 sensor wait: "
                    f"bmp_valid={int(bmp_valid)} bmp_stale={bmp_stale_sec:.2f}s "
                    f"acc_valid={int(acc_valid)} acc_stale={bno_acc_stale_sec:.2f}s "
                    f"alt={snapshot['alt']:.2f} fall={float(snapshot['fall'] or 0.0):.2f}"
                )

        if is_drop:
            if controller.phase0_drop_detect_time is None:
                controller.phase0_drop_detect_time = now
                print(
                    f"Detected Drop: {altitude_diff:.2f}m "
                    f"(waiting {PHASE0_DROP_TO_PHASE1_DELAY_SEC:.1f}s before Phase1)"
                )
            elif now - controller.phase0_drop_detect_time >= PHASE0_DROP_TO_PHASE1_DELAY_SEC:
                print(f"Drop hold complete: {altitude_diff:.2f}m -> Phase1")
                controller.st.update_navigation(phase=int(Phase.PHASE1))
                controller.time_phase1_start = now
                return
        else:
            if controller.phase0_drop_detect_time is not None:
                print("Drop hold canceled: altitude difference returned below threshold")
            controller.phase0_drop_detect_time = None

        if is_impact:
            print(f"Detected Impact: {snapshot['fall']:.2f}m/s^2")
            controller.st.update_navigation(phase=int(Phase.PHASE1))
            controller.time_phase1_start = now
            return

        if controller.time_phase1_start is None:
            phase0_start = entry_marker if entry_marker is not None else now
            if now - phase0_start > TIMEOUT_PHASE_0:
                print("Phase0 TIMEOUT: Force proceed (Sensor failure?)")
                controller.st.update_navigation(phase=int(Phase.PHASE1))
                controller.time_phase1_start = now


def _print_direct_run_help():
    print("p0.py is a phase handler module and does not run mission logic by itself.")
    print("Run mission from project root with: python3 main.py")


if __name__ == "__main__":
    _print_direct_run_help()
