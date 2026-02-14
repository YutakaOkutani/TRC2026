from cansat_mission.constants import Phase, TARGET_LAT, TARGET_LNG
from cansat_mission.controller import CanSatController


def run_full_mission(target_lat=TARGET_LAT, target_lng=TARGET_LNG):
    controller = CanSatController(target_lat, target_lng)
    controller.run(start_phase=Phase.PHASE0)


def run_phase_sequence(start_phase, allowed_phases, target_lat=TARGET_LAT, target_lng=TARGET_LNG):
    controller = CanSatController(target_lat, target_lng)
    controller.run(start_phase=start_phase, allowed_phases=allowed_phases)


def run_single_phase(phase, target_lat=TARGET_LAT, target_lng=TARGET_LNG):
    run_phase_sequence(
        start_phase=phase,
        allowed_phases=(phase,),
        target_lat=target_lat,
        target_lng=target_lng,
    )
