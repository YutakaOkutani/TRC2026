import time
import sys
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[2]
MAIN_PY_LIBRARY_DIR = PROJECT_ROOT / "library"
if not MAIN_PY_LIBRARY_DIR.exists():
    raise FileNotFoundError(f"main.py library directory not found: {MAIN_PY_LIBRARY_DIR}")
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from cansat_mission.constants import (
    CAMERA_DEAD_TIMEOUT,
    CAMERA_PHASE4_MAX_ATTEMPTS,
    CONE_CENTER_POSITION,
    CONE_PHASE4_CONFIRM_FRAMES,
    CONE_PHASE4_CENTER_TOLERANCE,
    CONE_PHASE4_STRONG_PROBABILITY,
    CONE_PROBABILITY_THRESHOLD,
    CONE_PROBABILITY_THRESHOLD_PHASE4,
    DEVICE_LED_GREEN,
    DEVICE_LED_RED,
    GPS_ACTIVE_DETECT,
    GPS_PHASE45_MAX_DISTANCE,
    Phase,
    TIMEOUT_PHASE_4,
)
from cansat_mission.navigation import calc_distance_and_azimuth
from cansat_mission.phases.base import BasePhaseHandler


class Phase4Handler(BasePhaseHandler):
    def _fallback_to_phase3(self, controller, current_snapshot, reason):
        fallback_dir = current_snapshot["angle"] if current_snapshot["angle_valid"] else current_snapshot["direction"]
        print(reason)
        controller.state.update_navigation(direction=fallback_dir, phase=int(Phase.PHASE3))
        controller.searching_flag = False
        controller.phase4_detect_confirm_count = 0
        controller.phase4_detect_confirm_marker = None
        controller.time_phase3_start = time.time()

    def execute(self, controller, snapshot):
        led_red = controller.devices.get(DEVICE_LED_RED)
        led_green = controller.devices.get(DEVICE_LED_GREEN)
        current_snapshot = controller.state.snapshot()
        cone_prob = current_snapshot["cone_probability"]
        cone_dir = current_snapshot.get("cone_direction", CONE_CENTER_POSITION)
        cone_reached = current_snapshot.get("cone_is_reached", False)
        print("phase4 : camera searching")
        if led_red:
            led_red.off()
        if led_green:
            led_green.on()
        if (
            hasattr(controller, "target_lat")
            and hasattr(controller, "target_lng")
            and current_snapshot.get("gps_detect") == GPS_ACTIVE_DETECT
        ):
            dist_m, azimuth = calc_distance_and_azimuth(
                current_snapshot["lat"],
                current_snapshot["lng"],
                controller.target_lat,
                controller.target_lng,
            )
            controller.state.update_navigation(distance=dist_m, azimuth=azimuth)
            if dist_m > GPS_PHASE45_MAX_DISTANCE:
                self._fallback_to_phase3(
                    controller,
                    current_snapshot,
                    f"Phase4 GPS fallback: target is {dist_m:.1f}m away (> {GPS_PHASE45_MAX_DISTANCE:.1f}m)",
                )
                return
        if not controller.searching_flag:
            controller.searching_flag = True
            controller.time_start_searching_cone = time.time()
            controller.camera_phase4_attempts += 1
            controller.camera_phase4_start = controller.time_start_searching_cone
        else:
            if time.time() - controller.time_start_searching_cone >= TIMEOUT_PHASE_4:
                print("Camera TIMEOUT: Cone not found or Camera dead")
                controller.searching_flag = False
                controller.state.update_navigation(phase=int(Phase.PHASE5))
                controller.time_phase5_start = time.time()
                return
        camera_dead = (
            controller.camera_dead_since is not None
            and time.time() - controller.camera_dead_since >= CAMERA_DEAD_TIMEOUT
        )
        if camera_dead and (
            controller.camera_phase4_attempts >= CAMERA_PHASE4_MAX_ATTEMPTS
            or (controller.camera_phase4_start is not None and time.time() - controller.camera_phase4_start >= TIMEOUT_PHASE_4)
        ):
            self._fallback_to_phase3(controller, current_snapshot, "Camera DEAD: Fallback to Phase3 (GPS/Straight)")
            return
        # Phase4開始時は比較的近距離(数m)のため、Phase5より厳しめの
        # probability と centered 条件で Phase5 へ直接渡す。
        try:
            cone_dir_val = float(cone_dir)
        except (TypeError, ValueError):
            cone_dir_val = CONE_CENTER_POSITION
        centered = abs(cone_dir_val - CONE_CENTER_POSITION) <= CONE_PHASE4_CENTER_TOLERANCE
        strict_detect = (cone_prob > CONE_PROBABILITY_THRESHOLD_PHASE4) and (
            centered or cone_prob >= CONE_PHASE4_STRONG_PROBABILITY
        )
        loose_detect = cone_prob > CONE_PROBABILITY_THRESHOLD

        if cone_reached:
            print("Phase4 -> Phase5: close-range visual reached detected")
            controller.searching_flag = False
            controller.phase4_detect_confirm_count = 0
            controller.phase4_detect_confirm_marker = None
            controller.state.update_navigation(phase=int(Phase.PHASE5))
            return

        if strict_detect:
            controller.phase4_detect_confirm_count = getattr(controller, "phase4_detect_confirm_count", 0) + 1
            if controller.phase4_detect_confirm_count >= CONE_PHASE4_CONFIRM_FRAMES:
                print(
                    f"Phase4 -> Phase5: confirmed cone detect x{controller.phase4_detect_confirm_count} "
                    f"(prob={cone_prob:.2f}, dir={cone_dir_val:.2f})"
                )
                controller.searching_flag = False
                controller.phase4_detect_confirm_count = 0
                controller.phase4_detect_confirm_marker = None
                controller.state.update_navigation(phase=int(Phase.PHASE5))
            elif controller.led_blink_timer % 5 == 0:
                print(
                    f"Phase4: holding visual confirm {controller.phase4_detect_confirm_count}/{CONE_PHASE4_CONFIRM_FRAMES} "
                    f"(prob={cone_prob:.2f}, dir={cone_dir_val:.2f})"
                )
            return

        controller.phase4_detect_confirm_count = 0
        controller.phase4_detect_confirm_marker = None

        if loose_detect and controller.led_blink_timer % 10 == 0:
            print(
                f"Phase4: low-confidence cone ignored (prob={cone_prob:.2f}, dir={cone_dir_val:.2f})"
            )


def run_standalone():
    from cansat_mission.runners import run_single_phase

    run_single_phase(Phase.PHASE4)


if __name__ == "__main__":
    run_standalone()
