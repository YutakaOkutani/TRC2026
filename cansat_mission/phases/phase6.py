import sys
import time
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[2]
MAIN_PY_LIBRARY_DIR = PROJECT_ROOT / "library"
if not MAIN_PY_LIBRARY_DIR.exists():
    raise FileNotFoundError(f"main.py library directory not found: {MAIN_PY_LIBRARY_DIR}")
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from cansat_mission.constants import DEVICE_LED_GREEN, DEVICE_LED_RED, PHASE6_RAM_DURATION_SEC, PHASE6_RAM_SPEED, Phase
from cansat_mission.phases.base import BasePhaseHandler


class Phase6Handler(BasePhaseHandler):
    def execute(self, controller, snapshot):
        led_red = controller.devices.get(DEVICE_LED_RED)
        led_green = controller.devices.get(DEVICE_LED_GREEN)
        entry_marker = getattr(controller, "phase_entry_time", None)
        if getattr(controller, "phase6_entry_marker", None) != entry_marker:
            controller.phase6_entry_marker = entry_marker
            controller.phase6_start_time = time.time()
            print(f"phase6 : final ram ({PHASE6_RAM_DURATION_SEC:.1f}s)")

        if led_red:
            led_red.on()
        if led_green:
            led_green.on()

        if getattr(controller, "mission_total_timeout_triggered", False):
            if getattr(controller, "mission_end_reason", "RUNNING") == "RUNNING":
                controller.mission_end_reason = "MISSION_TOTAL_TIMEOUT"
            controller.state.update_navigation(phase=int(Phase.PHASE7))
            return

        controller.set_motors(
            PHASE6_RAM_SPEED,
            True,
            PHASE6_RAM_SPEED,
            True,
            cmd_type="phase6_final_ram",
        )
        if time.time() - getattr(controller, "phase6_start_time", 0.0) >= PHASE6_RAM_DURATION_SEC:
            controller.state.update_navigation(phase=int(Phase.PHASE7))


def run_standalone():
    from cansat_mission.runners import run_single_phase

    run_single_phase(Phase.PHASE6)


if __name__ == "__main__":
    run_standalone()
