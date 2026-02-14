import os
import sys

ROOT_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if ROOT_DIR not in sys.path:
    sys.path.append(ROOT_DIR)

from cansat_mission.constants import Phase
from cansat_mission.runners import run_phase_sequence


def main():
    run_phase_sequence(
        start_phase=Phase.PHASE2,
        allowed_phases=(Phase.PHASE2, Phase.PHASE3),
    )


if __name__ == "__main__":
    main()
