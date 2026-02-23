import sys
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[2]
MAIN_PY_LIBRARY_DIR = PROJECT_ROOT / "library"
if not MAIN_PY_LIBRARY_DIR.exists():
    raise FileNotFoundError(f"main.py library directory not found: {MAIN_PY_LIBRARY_DIR}")
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from cansat_mission.constants import DEVICE_LED_GREEN, DEVICE_LED_RED, Phase
from cansat_mission.phases.base import BasePhaseHandler


class Phase7Handler(BasePhaseHandler):
    def execute(self, controller, snapshot):
        led_red = controller.devices.get(DEVICE_LED_RED)
        led_green = controller.devices.get(DEVICE_LED_GREEN)
        controller.stop_motors()
        if getattr(controller, "mission_total_timeout_triggered", False):
            if getattr(controller, "mission_end_reason", "RUNNING") == "RUNNING":
                controller.mission_end_reason = "MISSION_TOTAL_TIMEOUT"
            print("phase7 : Mission End (TOTAL TIMEOUT)")
            controller.signal_total_timeout_alert()
        else:
            if getattr(controller, "mission_end_reason", "RUNNING") == "RUNNING":
                controller.mission_end_reason = "PHASE7_EXIT"
            print("phase7 : Goal!! (stopped after final ram)")
            if led_red:
                led_red.on()
            if led_green:
                led_green.on()
        sys.exit()


def run_standalone():
    from cansat_mission.runners import run_single_phase

    run_single_phase(Phase.PHASE7)


if __name__ == "__main__":
    run_standalone()
