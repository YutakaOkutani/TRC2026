import sys
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]
MAIN_PY_LIBRARY_DIR = PROJECT_ROOT / "library"
if not MAIN_PY_LIBRARY_DIR.exists():
    raise FileNotFoundError(f"main.py library directory not found: {MAIN_PY_LIBRARY_DIR}")
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from cansat_mission.constants import Phase
from cansat_mission.runners import run_phase_sequence


def main():
    run_phase_sequence(
        start_phase=Phase.PHASE1,
        allowed_phases=(Phase.PHASE1, Phase.PHASE2, Phase.PHASE3, Phase.PHASE4, Phase.PHASE5, Phase.PHASE6, Phase.PHASE7),
    )


if __name__ == "__main__":
    main()
