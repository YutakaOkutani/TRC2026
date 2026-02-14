import time

from cansat_mission.constants import (
    CAMERA_DEAD_TIMEOUT,
    CAMERA_PHASE4_MAX_ATTEMPTS,
    CONE_PROBABILITY_THRESHOLD,
    DEVICE_LED_GREEN,
    DEVICE_LED_RED,
    Phase,
    TIMEOUT_PHASE_4,
)
from cansat_mission.phases.base import BasePhaseHandler


class Phase4Handler(BasePhaseHandler):
    def execute(self, controller, snapshot):
        led_red = controller.devices.get(DEVICE_LED_RED)
        led_green = controller.devices.get(DEVICE_LED_GREEN)
        current_snapshot = controller.state.snapshot()
        cone_prob = current_snapshot["cone_probability"]
        print("phase4 : camera searching")
        if led_red:
            led_red.off()
        if led_green:
            led_green.on()
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
        camera_dead = (
            controller.camera_dead_since is not None
            and time.time() - controller.camera_dead_since >= CAMERA_DEAD_TIMEOUT
        )
        if camera_dead and (
            controller.camera_phase4_attempts >= CAMERA_PHASE4_MAX_ATTEMPTS
            or (controller.camera_phase4_start is not None and time.time() - controller.camera_phase4_start >= TIMEOUT_PHASE_4)
        ):
            print("Camera DEAD: Fallback to Phase3 (GPS/Straight)")
            fallback_dir = current_snapshot["angle"] if current_snapshot["angle_valid"] else current_snapshot["direction"]
            controller.state.update_navigation(direction=fallback_dir, phase=int(Phase.PHASE3))
            controller.searching_flag = False
            controller.time_phase3_start = time.time()
            return
        if cone_prob > CONE_PROBABILITY_THRESHOLD:
            controller.state.update_navigation(phase=int(Phase.PHASE5))


def run_standalone():
    from cansat_mission.runners import run_single_phase

    run_single_phase(Phase.PHASE4)


if __name__ == "__main__":
    run_standalone()
