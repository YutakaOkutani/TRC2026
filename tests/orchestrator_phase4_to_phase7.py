import sys
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]
MAIN_PY_LIBRARY_DIR = PROJECT_ROOT / "library"
TEST_LOG_DIR = PROJECT_ROOT / "tests" / "log"
if not MAIN_PY_LIBRARY_DIR.exists():
    raise FileNotFoundError(f"main.py library directory not found: {MAIN_PY_LIBRARY_DIR}")
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from cansat_mission import constants as mission_constants
from cansat_mission import controller as mission_controller
from cansat_mission.constants import Phase
from cansat_mission.log_sync_launcher import trigger_async_log_sync
from cansat_mission.runners import run_phase_sequence


def main():
    # Reuse the production CSV logger format while saving under tests/log.
    mission_constants.LOG_DIR = str(TEST_LOG_DIR)
    mission_controller.LOG_DIR = str(TEST_LOG_DIR)
    try:
        run_phase_sequence(
            start_phase=Phase.PHASE4,
            allowed_phases=(Phase.PHASE4, Phase.PHASE5, Phase.PHASE6, Phase.PHASE7),
        )
    finally:
        trigger_async_log_sync("orchestrator_phase4_to_phase7_end")


if __name__ == "__main__":
    main()
