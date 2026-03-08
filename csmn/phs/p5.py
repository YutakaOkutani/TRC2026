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
    CAMERA_DEAD_TIMEOUT,
    CAMERA_PHASE5_MAX_ATTEMPTS,
    CONE_LOST_COUNT_LIMIT,
    CONE_PROBABILITY_THRESHOLD,
    CONE_PROBABILITY_THRESHOLD_PHASE5,
    DEVICE_LED_GREEN,
    DEVICE_LED_RED,
    GPS_ACTIVE_DETECT,
    GPS_PHASE45_MAX_DISTANCE,
    LED_INTERVAL_PHASE5,
    Phase,
    TIMEOUT_PHASE_5_AFTER_PHASE4_TIMEOUT,
    TIMEOUT_PHASE_5,
)
from csmn.nav import calc_distance_and_azimuth
from csmn.phs.base import BasePhaseHandler

PHASE5_REACH_CONFIRM_FRAMES = 8


class Phase5Handler(BasePhaseHandler):
    def _fallback_to_p3(self, controller, current_snapshot, reason):
        fallback_dir = current_snapshot["angle"] if current_snapshot["angle_valid"] else current_snapshot["direction"]
        print(reason)
        controller.st.update_navigation(direction=fallback_dir, phase=int(Phase.PHASE3))
        controller.time_phase3_start = time.time()

    def execute(self, controller, snapshot):
        led_red = controller.devices.get(DEVICE_LED_RED)
        led_green = controller.devices.get(DEVICE_LED_GREEN)
        entry_marker = getattr(controller, "phase_entry_time", None)
        need_phase5_init = False
        if entry_marker is not None:
            need_phase5_init = getattr(controller, "phase5_entry_marker", None) != entry_marker
        else:
            need_phase5_init = getattr(controller, "time_camera_start", 0.0) <= 0.0
        if need_phase5_init:
            entry_reason = str(getattr(controller, "phase5_entry_reason", "unknown"))
            timeout_limit = (
                TIMEOUT_PHASE_5_AFTER_PHASE4_TIMEOUT
                if entry_reason == "phase4_timeout"
                else TIMEOUT_PHASE_5
            )
            print(f"p5 : approaching ({entry_reason}, timeout={timeout_limit:.1f}s)")
            controller.phase5_entry_marker = entry_marker
            controller.phase5_timeout_limit_sec = float(timeout_limit)
            controller.time_camera_start = time.time()
            controller.count_cone_lost = 0
            controller.phase5_reach_confirm_count = 0
            controller.camera_phase5_attempts += 1
            controller.camera_phase5_start = controller.time_camera_start
        timeout_limit = float(getattr(controller, "phase5_timeout_limit_sec", TIMEOUT_PHASE_5))

        controller.led_blink_timer += 1
        if (controller.led_blink_timer // LED_INTERVAL_PHASE5) % 2 == 0:
            if led_red:
                led_red.on()
            if led_green:
                led_green.off()
        else:
            if led_red:
                led_red.off()
            if led_green:
                led_green.on()

        current_snapshot = controller.st.snapshot()
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
            controller.st.update_navigation(distance=dist_m, azimuth=azimuth)
            if dist_m > GPS_PHASE45_MAX_DISTANCE:
                self._fallback_to_p3(
                    controller,
                    current_snapshot,
                    f"Phase5 GPS fallback: target is {dist_m:.1f}m away (> {GPS_PHASE45_MAX_DISTANCE:.1f}m)",
                )
                return
        is_reach = current_snapshot["cone_is_reached"]
        # 近距離時はprobabilityが落ちても、到達判定が立っていれば見失い扱いにしない
        cone_prob = current_snapshot["cone_probability"]
        is_reach_effective = bool(is_reach) and (cone_prob > max(CONE_PROBABILITY_THRESHOLD_PHASE5, 0.30))
        is_det = (cone_prob > CONE_PROBABILITY_THRESHOLD_PHASE5) or is_reach_effective
        now = time.time()
        camera_dead = (
            controller.camera_dead_since is not None
            and now - controller.camera_dead_since >= CAMERA_DEAD_TIMEOUT
        )
        if camera_dead and (
            controller.camera_phase5_attempts >= CAMERA_PHASE5_MAX_ATTEMPTS
            or (controller.camera_phase5_start is not None and now - controller.camera_phase5_start >= timeout_limit)
        ):
            self._fallback_to_p3(controller, current_snapshot, "Camera DEAD: Fallback to Phase3 (GPS/Straight)")
            return

        if not is_det:
            controller.count_cone_lost += 1
            controller.phase5_reach_confirm_count = 0
            if cone_prob > CONE_PROBABILITY_THRESHOLD and controller.led_blink_timer % 10 == 0:
                print(f"Phase5: weak visual ignored (prob={cone_prob:.2f})")
        else:
            controller.count_cone_lost = 0

        if controller.count_cone_lost >= CONE_LOST_COUNT_LIMIT:
            print("Phase5 -> Phase4: cone lost")
            controller.st.update_navigation(phase=int(Phase.PHASE4))
            return

        if is_reach_effective:
            controller.phase5_reach_confirm_count = getattr(controller, "phase5_reach_confirm_count", 0) + 1
            if controller.phase5_reach_confirm_count < PHASE5_REACH_CONFIRM_FRAMES:
                return
            print(
                f"Reached Cone! (Visual confirmation x{controller.phase5_reach_confirm_count})"
            )
            controller.mission_end_reason = "GOAL_REACHED"
            controller.st.update_navigation(phase=int(Phase.PHASE6))
            return
        else:
            controller.phase5_reach_confirm_count = 0

        if now - controller.time_camera_start >= timeout_limit:
            elapsed = now - controller.time_camera_start
            print(f"Phase5 TIMEOUT ({elapsed:.1f}s / {timeout_limit:.1f}s): Giving up, forcing Goal")
            controller.mission_end_reason = "PHASE5_TIMEOUT_FORCED_GOAL"
            controller.st.update_navigation(phase=int(Phase.PHASE6))
            return


def run_standalone():
    from csmn.run import run_single_phase

    run_single_phase(Phase.PHASE5)


if __name__ == "__main__":
    run_standalone()
