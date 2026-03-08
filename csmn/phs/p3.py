import time
import sys
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[2]
MAIN_PY_LIBRARY_DIR = PROJECT_ROOT / "lib"
if not MAIN_PY_LIBRARY_DIR.exists():
    raise FileNotFoundError(f"main.py library directory not found: {MAIN_PY_LIBRARY_DIR}")
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from csmn.const import (
    DEVICE_LED_GREEN,
    DEVICE_LED_RED,
    GPS_ACTIVE_DETECT,
    GPS_CLOSE_DISTANCE,
    GPS_LOST_LOG_INTERVAL,
    LED_INTERVAL_PHASE3,
    PHASE_LOG_INTERVAL,
    Phase,
    TIMEOUT_PHASE_3,
)
from csmn.nav import calc_distance_and_azimuth
from csmn.phs.base import BasePhaseHandler


class Phase3Handler(BasePhaseHandler):
    def execute(self, controller, snapshot):
        led_red = controller.devices.get(DEVICE_LED_RED)
        led_green = controller.devices.get(DEVICE_LED_GREEN)
        if led_red:
            led_red.off()
        controller.toggle_led(led_green, controller.led_blink_timer, interval=LED_INTERVAL_PHASE3)
        if time.time() - controller.time_phase3_start > TIMEOUT_PHASE_3:
            print("Phase3 TIMEOUT: switching to Phase4")
            controller.st.update_navigation(phase=int(Phase.PHASE4))
            controller.time_phase4_start = time.time()
            return
        if snapshot["gps_detect"] == GPS_ACTIVE_DETECT:
            dist, azimuth = calc_distance_and_azimuth(snapshot["lat"], snapshot["lng"], controller.target_lat, controller.target_lng)
            controller.st.update_navigation(distance=dist, azimuth=azimuth, direction=azimuth)
            nav_heading = None
            nav_heading_source = "INVALID"
            if hasattr(controller, "_phase3_heading"):
                nav_heading, nav_heading_source = controller._phase3_heading(snapshot)
            elif hasattr(controller, "_phase45_bno_heading"):
                nav_heading = controller._phase45_bno_heading(snapshot)
                nav_heading_source = "BNO" if nav_heading is not None else "INVALID"
            heading_diff = None
            if nav_heading is not None:
                heading_diff = controller._angle_diff_deg(azimuth, nav_heading)
            if controller.led_blink_timer % PHASE_LOG_INTERVAL == 0:
                if nav_heading is not None and heading_diff is not None:
                    print(
                        f"GPS Nav: Dist={dist:.1f}m, TargetDir={azimuth:.1f}, "
                        f"Head({nav_heading_source})={nav_heading:.1f}, Diff={heading_diff:+.1f}"
                    )
                else:
                    print(
                        f"GPS Nav: Dist={dist:.1f}m, TargetDir={azimuth:.1f}, "
                        f"Head(INVALID:{nav_heading_source}), "
                        f"AngleValid={int(bool(snapshot.get('angle_valid', False)))}, "
                        f"GPSHeadingValid={int(bool(snapshot.get('gps_heading_valid', False)))}, "
                        f"GPSSpeed={float(snapshot.get('gps_speed_mps', 0.0)):.2f}m/s, "
                        f"BNOStale={float(getattr(controller, 'bno_stale_sec', 0.0)):.2f}s"
                    )
            if dist < GPS_CLOSE_DISTANCE:
                print(f"Close enough ({dist:.1f}m): switching to Phase4")
                controller.st.update_navigation(phase=int(Phase.PHASE4))
                controller.time_phase4_start = time.time()
        else:
            if controller.led_blink_timer % GPS_LOST_LOG_INTERVAL == 0:
                print("GPS Lost: Keep going...")


def run_standalone():
    from csmn.run import run_single_phase

    run_single_phase(Phase.PHASE3)


if __name__ == "__main__":
    run_standalone()
