import sys
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[2]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from runs.orch.orch_p0_log import run_phase0_logging


def main():
    run_phase0_logging("open_parachute")


if __name__ == "__main__":
    main()
