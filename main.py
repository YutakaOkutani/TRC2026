from cansat_mission.runners import run_full_mission
from cansat_mission.log_sync_launcher import trigger_async_log_sync


def main():
    try:
        run_full_mission()
    finally:
        trigger_async_log_sync("main.py_end")


if __name__ == "__main__":
    main()
