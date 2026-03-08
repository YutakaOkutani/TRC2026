from csmn import const as mission_constants
from csmn.const import Phase
from csmn.ctrl import CanSatController


def _resolve_target(target_lat, target_lng):
    if target_lat is None:
        target_lat = mission_constants.TARGET_LAT
    if target_lng is None:
        target_lng = mission_constants.TARGET_LNG
    return float(target_lat), float(target_lng)


def run_full_mission(target_lat=None, target_lng=None):
    target_lat, target_lng = _resolve_target(target_lat, target_lng)
    controller = CanSatController(target_lat, target_lng)
    controller.run(start_phase=Phase.PHASE0)


def run_phase_sequence(start_phase, allowed_phases, target_lat=None, target_lng=None):
    target_lat, target_lng = _resolve_target(target_lat, target_lng)
    controller = CanSatController(target_lat, target_lng)
    controller.run(start_phase=start_phase, allowed_phases=allowed_phases)


def run_single_phase(phase, target_lat=None, target_lng=None):
    run_phase_sequence(
        start_phase=phase,
        allowed_phases=(phase,),
        target_lat=target_lat,
        target_lng=target_lng,
    )
