import time

from cansat_mission.constants import (
    DEVICE_LED_GREEN,
    DEVICE_LED_RED,
    PARACHUTE_DIRECTION,
    PHASE2_STAGE_STRAIGHT,
    Phase,
    TIMEOUT_PHASE_1,
)
from cansat_mission.phases.base import BasePhaseHandler


class Phase1Handler(BasePhaseHandler):
    def execute(self, controller, snapshot):
        led_red = controller.devices.get(DEVICE_LED_RED)
        led_green = controller.devices.get(DEVICE_LED_GREEN)
        if led_red:
            led_red.on()
        if led_green:
            led_green.off()
        print("PH1: Start Parachute Separation")
        if controller.time_phase1_start is None:
            controller.time_phase1_start = time.time()
        elapsed = time.time() - controller.time_phase1_start
        if elapsed < TIMEOUT_PHASE_1:
            controller.state.update_navigation(direction=PARACHUTE_DIRECTION, phase=int(Phase.PHASE1))
            return
        print("PH1: Parachute Separation TIMEOUT -> switching to Phase2")
        controller.state.update_navigation(phase=int(Phase.PHASE2))
        controller.phase2_start_time = time.time()
        controller.phase2_stage = PHASE2_STAGE_STRAIGHT
        controller.phase2_stage_start = controller.phase2_start_time
        controller.time_phase1_start = None


def run_standalone():
    from cansat_mission.runners import run_single_phase

    run_single_phase(Phase.PHASE1)


if __name__ == "__main__":
    run_standalone()
