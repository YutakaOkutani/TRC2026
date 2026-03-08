import sys
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[2]
MAIN_PY_LIBRARY_DIR = PROJECT_ROOT / "lib"
TEST_LOG_ROOT = PROJECT_ROOT / "runs" / "log"
if not MAIN_PY_LIBRARY_DIR.exists():
    raise FileNotFoundError(f"main.py library directory not found: {MAIN_PY_LIBRARY_DIR}")
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from csmn import const as mission_constants
from csmn import ctrl as mission_controller
from csmn.const import Phase
from csmn.run import run_phase_sequence


def run_phase0_logging(log_subdir: str) -> None:
    log_dir = TEST_LOG_ROOT / log_subdir
    # Use the same logger path and schema path as main.py via CanSatController.
    mission_constants.LOG_DIR = str(log_dir)
    mission_controller.LOG_DIR = str(log_dir)
    run_phase_sequence(
        start_phase=Phase.PHASE0,
        allowed_phases=(Phase.PHASE0,),
    )
