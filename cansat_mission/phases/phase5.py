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
    CAMERA_PHASE5_MAX_ATTEMPTS,
    CONE_LOST_COUNT_LIMIT,
    CONE_PROBABILITY_THRESHOLD,
    DEVICE_LED_GREEN,
    DEVICE_LED_RED,
    LED_INTERVAL_PHASE5,
    Phase,
    TIMEOUT_PHASE_5,
)
from cansat_mission.phases.base import BasePhaseHandler

PHASE5_REACH_CONFIRM_FRAMES = 4


class Phase5Handler(BasePhaseHandler):
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
            print("phase5 : approaching")
            controller.phase5_entry_marker = entry_marker
            controller.time_camera_start = time.time()
            controller.count_cone_lost = 0
            controller.phase5_reach_confirm_count = 0
            controller.camera_phase5_attempts += 1
            controller.camera_phase5_start = controller.time_camera_start

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

        current_snapshot = controller.state.snapshot()
        is_reach = current_snapshot["cone_is_reached"]
        # 近距離時はprobabilityが落ちても、到達判定が立っていれば見失い扱いにしない
        is_det = (current_snapshot["cone_probability"] > CONE_PROBABILITY_THRESHOLD) or is_reach
        now = time.time()
        camera_dead = (
            controller.camera_dead_since is not None
            and now - controller.camera_dead_since >= CAMERA_DEAD_TIMEOUT
        )
        if camera_dead and (
            controller.camera_phase5_attempts >= CAMERA_PHASE5_MAX_ATTEMPTS
            or (controller.camera_phase5_start is not None and now - controller.camera_phase5_start >= TIMEOUT_PHASE_5)
        ):
            print("Camera DEAD: Fallback to Phase3 (GPS/Straight)")
            fallback_dir = current_snapshot["angle"] if current_snapshot["angle_valid"] else current_snapshot["direction"]
            controller.state.update_navigation(direction=fallback_dir, phase=int(Phase.PHASE3))
            return

        if not is_det:
            controller.count_cone_lost += 1
            controller.phase5_reach_confirm_count = 0
        else:
            controller.count_cone_lost = 0

        if controller.count_cone_lost >= CONE_LOST_COUNT_LIMIT:
            print("Phase5 -> Phase4: cone lost")
            controller.state.update_navigation(phase=int(Phase.PHASE4))
            return

        if is_reach:
            controller.phase5_reach_confirm_count = getattr(controller, "phase5_reach_confirm_count", 0) + 1
            if controller.phase5_reach_confirm_count < PHASE5_REACH_CONFIRM_FRAMES:
                return
            print(
                f"Reached Cone! (Visual confirmation x{controller.phase5_reach_confirm_count})"
            )
            controller.mission_end_reason = "GOAL_REACHED"
            controller.state.update_navigation(phase=int(Phase.PHASE6))
            return
        else:
            controller.phase5_reach_confirm_count = 0

        if now - controller.time_camera_start >= TIMEOUT_PHASE_5:
            elapsed = now - controller.time_camera_start
            print(f"Phase5 TIMEOUT ({elapsed:.1f}s): Giving up, forcing Goal")
            controller.mission_end_reason = "PHASE5_TIMEOUT_FORCED_GOAL"
            controller.state.update_navigation(phase=int(Phase.PHASE6))
            return


def run_standalone():
    from cansat_mission.runners import run_single_phase

    run_single_phase(Phase.PHASE5)


if __name__ == "__main__":
    run_standalone()
