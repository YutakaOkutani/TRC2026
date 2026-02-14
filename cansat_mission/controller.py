import datetime
import math
import os
import sys
import time

from cansat_mission.constants import (
    DEFAULT_BNO_CALIB,
    DEFAULT_VECTOR3,
    DEVICE_KEYS,
    GPS_HEADING_OFFSET,
    HEADING_MAG_CALIB_MAX,
    HEADING_SOURCE_BNO,
    HEADING_SOURCE_GPS,
    HEADING_SOURCE_INVALID,
    HEADING_SOURCE_JOINER,
    HEADING_WEIGHT_BNO_BASE,
    HEADING_WEIGHT_BNO_MAX,
    HEADING_WEIGHT_BNO_MIN,
    HEADING_WEIGHT_BNO_STEP,
    HEADING_WEIGHT_GPS,
    LED_SIGNAL_COUNT,
    LOG_DIR,
    LOG_FILE_DATETIME_FORMAT,
    LOG_PREFIX,
    MAIN_LOOP_INTERVAL,
    PHASE2_STAGE_STRAIGHT,
    Phase,
)
from cansat_mission.managers import HardwareManager, LedManager, MotorManager, SensorManager
from cansat_mission.phases import (
    Phase0Handler,
    Phase1Handler,
    Phase2Handler,
    Phase3Handler,
    Phase4Handler,
    Phase5Handler,
    Phase6Handler,
)
from cansat_mission.state import CanSatState


class CanSatController(HardwareManager, SensorManager, MotorManager, LedManager):
    def __init__(self, target_lat, target_lng):
        self.state = CanSatState()
        self.target_lat = target_lat
        self.target_lng = target_lng
        now_time = datetime.datetime.now()
        os.makedirs(LOG_DIR, exist_ok=True)
        self.log_path = os.path.join(LOG_DIR, LOG_PREFIX + now_time.strftime(LOG_FILE_DATETIME_FORMAT) + ".csv")

        self.devices = {key: None for key in DEVICE_KEYS}
        self.led_blink_timer = 0
        self.searching_flag = False
        self.count_cone_lost = 0
        self.time_phase1_start = None
        self.time_phase3_start = 0.0
        self.time_phase4_start = 0.0
        self.time_phase5_start = 0.0
        self.time_start_searching_cone = 0.0
        self.time_camera_start = 0.0
        self.motor_state = {}
        self.bno_fail_count = 0
        self.bno_last_reinit_time = 0.0
        self.bno_last_valid = {
            "acc": list(DEFAULT_VECTOR3),
            "gyro": list(DEFAULT_VECTOR3),
            "mag": list(DEFAULT_VECTOR3),
            "angle": 0.0,
        }
        self.bno_last_valid_time = 0.0
        self.bno_stale_sec = 0.0
        self.bno_calib = dict(DEFAULT_BNO_CALIB)
        self.phase2_start_time = None
        self.phase2_stage = PHASE2_STAGE_STRAIGHT
        self.phase2_stage_start = None
        self.roi_img = None
        self.camera_fail_count = 0
        self.camera_last_reinit = 0.0
        self.camera_dead_since = None
        self.camera_phase4_attempts = 0
        self.camera_phase5_attempts = 0
        self.camera_phase4_start = None
        self.camera_phase5_start = None
        self.obstacle_detect_count = 0
        self.phase3_no_heading_start = None

        self.phase_handlers = {
            Phase.PHASE0: Phase0Handler(),
            Phase.PHASE1: Phase1Handler(),
            Phase.PHASE2: Phase2Handler(),
            Phase.PHASE3: Phase3Handler(),
            Phase.PHASE4: Phase4Handler(),
            Phase.PHASE5: Phase5Handler(),
            Phase.PHASE6: Phase6Handler(),
        }

    def initialize_phase(self, phase):
        phase_enum = Phase(phase)
        now = time.time()
        self.state.update_navigation(phase=int(phase_enum))
        if phase_enum == Phase.PHASE1:
            self.time_phase1_start = now
        elif phase_enum == Phase.PHASE2:
            self.phase2_start_time = now
            self.phase2_stage = PHASE2_STAGE_STRAIGHT
            self.phase2_stage_start = now
        elif phase_enum == Phase.PHASE3:
            self.time_phase3_start = now
        elif phase_enum == Phase.PHASE4:
            self.time_phase4_start = now
            self.searching_flag = False
        elif phase_enum == Phase.PHASE5:
            self.time_phase5_start = now

    def run(self, start_phase=Phase.PHASE0, allowed_phases=None):
        self.setup_hardware()
        self.signal_led(LED_SIGNAL_COUNT)
        self.initialize_phase(start_phase)
        allowed_set = None
        if allowed_phases is not None:
            allowed_set = {Phase(value) for value in allowed_phases}
        try:
            while True:
                if allowed_set is not None:
                    current_phase = Phase(self.state.snapshot()["phase"])
                    if current_phase not in allowed_set:
                        print(f"Phase subset completed at phase {int(current_phase)}")
                        self.stop_motors()
                        return
                self.loop_once()
                time.sleep(MAIN_LOOP_INTERVAL)
        except KeyboardInterrupt:
            print("\nKeyboardInterrupt")
            self.stop_motors()
            sys.exit()

    def loop_once(self):
        snapshot = self.state.snapshot()
        phase = Phase(snapshot["phase"])
        self.led_blink_timer += 1
        handler = self.phase_handlers.get(phase)
        if handler is not None:
            handler.execute(self, snapshot)

    def _angle_diff_deg(self, target_deg, current_deg):
        diff = target_deg - current_deg
        if diff > 180.0:
            diff -= 360.0
        if diff < -180.0:
            diff += 360.0
        return diff

    def _weighted_heading(self, snapshot):
        weights = []
        headings = []
        source_parts = []

        if snapshot.get("gps_heading_valid", False):
            weights.append(HEADING_WEIGHT_GPS)
            headings.append(snapshot.get("gps_heading", 0.0))
            source_parts.append(HEADING_SOURCE_GPS)

        if snapshot.get("angle_valid", False):
            bno_weight = HEADING_WEIGHT_BNO_BASE
            calib = self.bno_calib
            if calib.get("valid") and len(calib.get("value", ())) >= 4:
                try:
                    mag_cal = int(calib["value"][3])
                    bno_weight += HEADING_WEIGHT_BNO_STEP * max(0, min(HEADING_MAG_CALIB_MAX, mag_cal))
                except Exception:
                    pass
            bno_weight = max(HEADING_WEIGHT_BNO_MIN, min(HEADING_WEIGHT_BNO_MAX, bno_weight))
            weights.append(bno_weight)
            headings.append((snapshot.get("angle", 0.0) + GPS_HEADING_OFFSET) % 360.0)
            source_parts.append(HEADING_SOURCE_BNO)

        if not headings:
            return None, HEADING_SOURCE_INVALID, 0.0

        sx = 0.0
        sy = 0.0
        total_w = 0.0
        for heading, weight in zip(headings, weights):
            rad = math.radians(heading)
            sx += math.cos(rad) * weight
            sy += math.sin(rad) * weight
            total_w += weight
        if total_w <= 0.0:
            return None, HEADING_SOURCE_INVALID, 0.0
        fused = math.degrees(math.atan2(sy, sx))
        if fused < 0.0:
            fused += 360.0
        source = HEADING_SOURCE_JOINER.join(source_parts)
        return fused, source, total_w
