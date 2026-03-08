import sys
import os
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[2]
MAIN_PY_LIBRARY_DIR = PROJECT_ROOT / "lib"
TEST_LOG_DIR = PROJECT_ROOT / "runs" / "log"
if not MAIN_PY_LIBRARY_DIR.exists():
    raise FileNotFoundError(f"main.py library directory not found: {MAIN_PY_LIBRARY_DIR}")
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from csmn import const as mission_constants
from csmn import ctrl as mission_controller
from csmn.const import Phase
from csmn.run import run_phase_sequence


def main():
    # Reuse the production CSV logger format while saving under runs/log.
    mission_constants.LOG_DIR = str(TEST_LOG_DIR)
    mission_controller.LOG_DIR = str(TEST_LOG_DIR)
    target_lat = float(os.getenv("CANSAT_TARGET_LAT", mission_constants.TARGET_LAT))
    target_lng = float(os.getenv("CANSAT_TARGET_LNG", mission_constants.TARGET_LNG))
    print(f"Phase2-7 test target: lat={target_lat:.6f}, lng={target_lng:.6f}")
    run_phase_sequence(
        start_phase=Phase.PHASE2,
        allowed_phases=(Phase.PHASE2, Phase.PHASE3, Phase.PHASE4, Phase.PHASE5, Phase.PHASE6, Phase.PHASE7),
        target_lat=target_lat,
        target_lng=target_lng,
    )


if __name__ == "__main__":
    main()
