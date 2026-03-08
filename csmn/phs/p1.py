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
    PARACHUTE_DIRECTION,
    PHASE2_STAGE_STRAIGHT,
    Phase,
    TIMEOUT_PHASE_1,
)
from csmn.phs.base import BasePhaseHandler


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
            controller.st.update_navigation(direction=PARACHUTE_DIRECTION, phase=int(Phase.PHASE1))
            return
        print("PH1: Parachute Separation TIMEOUT -> switching to Phase2")
        controller.st.update_navigation(phase=int(Phase.PHASE2))
        controller.phase2_start_time = time.time()
        controller.phase2_stage = PHASE2_STAGE_STRAIGHT
        controller.phase2_stage_start = controller.phase2_start_time
        controller.time_phase1_start = None


def run_standalone():
    from csmn.run import run_single_phase

    run_single_phase(Phase.PHASE1)


if __name__ == "__main__":
    run_standalone()
