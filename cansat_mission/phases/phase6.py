import sys

from cansat_mission.constants import DEVICE_LED_GREEN, DEVICE_LED_RED, Phase
from cansat_mission.phases.base import BasePhaseHandler


class Phase6Handler(BasePhaseHandler):
    def execute(self, controller, snapshot):
        led_red = controller.devices.get(DEVICE_LED_RED)
        led_green = controller.devices.get(DEVICE_LED_GREEN)
        print("phase6 : Goal!!")
        if led_red:
            led_red.on()
        if led_green:
            led_green.on()
        controller.stop_motors()
        sys.exit()


def run_standalone():
    from cansat_mission.runners import run_single_phase

    run_single_phase(Phase.PHASE6)


if __name__ == "__main__":
    run_standalone()
