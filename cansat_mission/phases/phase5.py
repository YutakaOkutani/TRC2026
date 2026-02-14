import time

from cansat_mission.constants import (
    CAMERA_DEAD_TIMEOUT,
    CAMERA_PHASE5_MAX_ATTEMPTS,
    CONE_LOST_COUNT_LIMIT,
    CONE_PROBABILITY_THRESHOLD,
    DEVICE_LED_GREEN,
    DEVICE_LED_RED,
    LED_INTERVAL_PHASE5,
    Phase,
    SHORT_SLEEP,
    TIMEOUT_PHASE_5,
)
from cansat_mission.phases.base import BasePhaseHandler


class Phase5Handler(BasePhaseHandler):
    def execute(self, controller, snapshot):
        led_red = controller.devices.get(DEVICE_LED_RED)
        led_green = controller.devices.get(DEVICE_LED_GREEN)
        print("phase5 : approaching")
        controller.time_camera_start = time.time()
        controller.count_cone_lost = 0
        controller.camera_phase5_attempts += 1
        controller.camera_phase5_start = controller.time_camera_start
        while True:
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
            is_det = current_snapshot["cone_probability"] > CONE_PROBABILITY_THRESHOLD
            is_reach = current_snapshot["cone_is_reached"]
            camera_dead = (
                controller.camera_dead_since is not None
                and time.time() - controller.camera_dead_since >= CAMERA_DEAD_TIMEOUT
            )
            if camera_dead and (
                controller.camera_phase5_attempts >= CAMERA_PHASE5_MAX_ATTEMPTS
                or (controller.camera_phase5_start is not None and time.time() - controller.camera_phase5_start >= TIMEOUT_PHASE_5)
            ):
                print("Camera DEAD: Fallback to Phase3 (GPS/Straight)")
                fallback_dir = current_snapshot["angle"] if current_snapshot["angle_valid"] else current_snapshot["direction"]
                controller.state.update_navigation(direction=fallback_dir, phase=int(Phase.PHASE3))
                break
            if not is_det:
                controller.count_cone_lost += 1
            else:
                controller.count_cone_lost = 0
            if controller.count_cone_lost >= CONE_LOST_COUNT_LIMIT:
                controller.state.update_navigation(phase=int(Phase.PHASE4))
                break
            if time.time() - controller.time_camera_start >= TIMEOUT_PHASE_5:
                print("Phase5 TIMEOUT: Giving up, forcing Goal")
                controller.state.update_navigation(phase=int(Phase.PHASE6))
                break
            if is_reach:
                print("Reached Cone! (Visual confirmation)")
                controller.state.update_navigation(phase=int(Phase.PHASE6))
                break
            time.sleep(SHORT_SLEEP)


def run_standalone():
    from cansat_mission.runners import run_single_phase

    run_single_phase(Phase.PHASE5)


if __name__ == "__main__":
    run_standalone()
