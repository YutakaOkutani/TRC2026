import datetime
import math
import os
import time
import csv

from csmn.const import (
    DEFAULT_BNO_CALIB,
    DEFAULT_VECTOR3,
    DEVICE_KEYS,
    HEADING_OFFSET_LEARN_BOOTSTRAP_SAMPLES,
    HEADING_OFFSET_LEARN_MAX_RESIDUAL_DEG,
    HEADING_OFFSET_LEARN_MIN_SPEED_MPS,
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
    MISSION_PHASE_TIME_BUDGETS,
    MISSION_PHASE_TIMEOUT_TRANSITIONS,
    MISSION_TIMEOUT_TOTAL,
    PHASE3_BNO_GPS_OFFSET_ALPHA,
    PHASE3_BNO_GPS_OFFSET_MAX_STEP_DEG,
    PHASE2_STAGE_STRAIGHT,
    Phase,
)
from csmn.mgr import HardwareManager, LedManager, MotorManager, SensorManager
from csmn.phs import (
    Phase0Handler,
    Phase1Handler,
    Phase2Handler,
    Phase3Handler,
    Phase4Handler,
    Phase5Handler,
    Phase6Handler,
    Phase7Handler,
)
from csmn.st import CanSatState


class CanSatController(HardwareManager, SensorManager, MotorManager, LedManager):
    @staticmethod
    def _build_unique_log_path(log_dir, now_time):
        log_stem = LOG_PREFIX + now_time.strftime(LOG_FILE_DATETIME_FORMAT) + f"-{now_time.microsecond:06d}"
        log_path = os.path.join(log_dir, log_stem + ".csv")
        if not os.path.exists(log_path):
            return log_path
        suffix = 1
        while True:
            candidate = os.path.join(log_dir, f"{log_stem}-{suffix}.csv")
            if not os.path.exists(candidate):
                return candidate
            suffix += 1

    def __init__(self, target_lat, target_lng):
        self.st = CanSatState()
        self.target_lat = target_lat
        self.target_lng = target_lng
        now_time = datetime.datetime.now()
        os.makedirs(LOG_DIR, exist_ok=True)
        self.log_path = self._build_unique_log_path(LOG_DIR, now_time)

        self.devices = {key: None for key in DEVICE_KEYS}
        self.led_blink_timer = 0
        self.searching_flag = False
        self.count_cone_lost = 0
        self.time_phase1_start = None
        self.phase0_entry_marker = None
        self.phase0_initial_alt = None
        self.phase0_drop_detect_time = None
        self.time_phase3_start = 0.0
        self.time_phase4_start = 0.0
        self.time_phase5_start = 0.0
        self.time_start_searching_cone = 0.0
        self.time_camera_start = 0.0
        self.motor_state = {}
        self.last_motor_command = {
            "type": "init",
            "updated_ms": 0,
            "motor1_speed": 0.0,
            "motor1_forward": 1,
            "motor2_speed": 0.0,
            "motor2_forward": 1,
        }
        self.bno_fail_count = 0
        self.bno_last_reinit_time = 0.0
        self.bno_last_valid = {
            "acc": list(DEFAULT_VECTOR3),
            "gyro": list(DEFAULT_VECTOR3),
            "mag": list(DEFAULT_VECTOR3),
            "angle": 0.0,
        }
        self.bno_last_acc_time = 0.0
        self.bno_last_valid_time = 0.0
        self.bno_stale_sec = 0.0
        self.bno_acc_stale_sec = 0.0
        self.bno_calib = dict(DEFAULT_BNO_CALIB)
        self.bmp_last_valid_time = 0.0
        self.bmp_stale_sec = 0.0
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
        self.camera_candidate_last_dir = None
        self.camera_candidate_last_time = 0.0
        self.camera_candidate_streak = 0
        self.camera_stable_dir = None
        self.camera_stable_prob = 0.0
        self.camera_stable_seen_time = 0.0
        self.phase5_entry_marker = None
        self.phase5_entry_reason = "unknown"
        self.phase5_timeout_limit_sec = 0.0
        self.phase0_wait_log_counter = 0
        self.obstacle_detect_count = 0
        self.phase3_no_heading_start = None
        self.phase4_detect_confirm_count = 0
        self.phase4_detect_confirm_marker = None
        self.bno_heading_offset_deg = 0.0
        self.bno_heading_offset_valid = False
        self.bno_heading_offset_candidate_deg = None
        self.bno_heading_offset_candidate_count = 0
        self.mag_heading_offset_deg = 0.0
        self.mag_heading_offset_valid = False
        self.mag_heading_offset_candidate_deg = None
        self.mag_heading_offset_candidate_count = 0
        self.mission_start_time = None
        self.phase_entry_time = None
        self.last_phase_observed = None
        self.phase_elapsed_totals = {phase: 0.0 for phase in Phase}
        self.phase7_arrival_reason = "RUNNING"
        self.mission_end_reason = "RUNNING"
        self.mission_total_timeout_triggered = False
        self._shutdown_requested = False

        self.phase_handlers = {
            Phase.PHASE0: Phase0Handler(),
            Phase.PHASE1: Phase1Handler(),
            Phase.PHASE2: Phase2Handler(),
            Phase.PHASE3: Phase3Handler(),
            Phase.PHASE4: Phase4Handler(),
            Phase.PHASE5: Phase5Handler(),
            Phase.PHASE6: Phase6Handler(),
            Phase.PHASE7: Phase7Handler(),
        }

    def request_shutdown(self, reason="shutdown"):
        if self._shutdown_requested:
            return
        self._shutdown_requested = True
        if self.mission_end_reason == "RUNNING":
            self.mission_end_reason = reason
        if self.phase7_arrival_reason == "RUNNING" and self.st.snapshot().get("phase") == int(Phase.PHASE7):
            self.phase7_arrival_reason = self._resolve_phase7_arrival_reason()
        try:
            self.stop_motors()
        except Exception as exc:
            print(f"Emergency stop failed: {exc}")
        self._write_final_log_row()

    def _resolve_phase7_arrival_reason(self):
        reason = str(getattr(self, "mission_end_reason", "RUNNING"))
        if reason == "GOAL_REACHED":
            return "GOAL_REACHED"
        if reason == "MISSION_TOTAL_TIMEOUT":
            return "MISSION_TOTAL_TIMEOUT"
        if reason == "PHASE5_TIMEOUT_FORCED_GOAL":
            return "PHASE5_TIMEOUT_FORCED_GOAL"
        return "OTHER_ABNORMAL_EXIT"

    def _write_final_log_row(self):
        try:
            with open(self.log_path, "a", newline="") as file_obj:
                writer = csv.writer(file_obj)
                self._append_log_row(writer, file_obj)
        except Exception as exc:
            print(f"Final Log Error: {exc}")

    def initialize_phase(self, phase):
        phase_enum = Phase(phase)
        now = time.time()
        if self.mission_start_time is None:
            self.mission_start_time = now
        self.last_phase_observed = phase_enum
        self.phase_entry_time = now
        self.st.update_navigation(phase=int(phase_enum))
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
            self.phase4_detect_confirm_count = 0
            self.phase4_detect_confirm_marker = None
        elif phase_enum == Phase.PHASE5:
            self.time_phase5_start = now

    def _sync_phase_time_tracking(self, current_phase):
        now = time.time()
        if self.mission_start_time is None:
            self.mission_start_time = now
        if self.last_phase_observed is None:
            self.last_phase_observed = current_phase
            self.phase_entry_time = now
            return
        if current_phase != self.last_phase_observed:
            if self.phase_entry_time is not None:
                self.phase_elapsed_totals[self.last_phase_observed] += max(0.0, now - self.phase_entry_time)
            self.last_phase_observed = current_phase
            self.phase_entry_time = now

    def _current_phase_elapsed(self, phase, now):
        elapsed = self.phase_elapsed_totals.get(phase, 0.0)
        if self.last_phase_observed == phase and self.phase_entry_time is not None:
            elapsed += max(0.0, now - self.phase_entry_time)
        return elapsed

    def _commit_active_phase_elapsed(self, phase, now):
        if self.last_phase_observed != phase or self.phase_entry_time is None:
            return
        self.phase_elapsed_totals[phase] += max(0.0, now - self.phase_entry_time)

    def _handle_timeout_transitions(self, current_phase):
        now = time.time()
        if self.mission_start_time is None:
            self.mission_start_time = now
        mission_elapsed = now - self.mission_start_time
        if mission_elapsed >= MISSION_TIMEOUT_TOTAL:
            if not self.mission_total_timeout_triggered:
                print(f"MISSION TIMEOUT ({mission_elapsed:.1f}s): forcing Phase7 give-up")
                self._commit_active_phase_elapsed(current_phase, now)
                self.mission_end_reason = "MISSION_TOTAL_TIMEOUT"
                self.mission_total_timeout_triggered = True
                self.initialize_phase(Phase.PHASE7)
                return True
            return current_phase != Phase.PHASE7

        phase_budget = MISSION_PHASE_TIME_BUDGETS.get(current_phase)
        if phase_budget is None:
            return False
        phase_elapsed = self._current_phase_elapsed(current_phase, now)
        if phase_elapsed < phase_budget:
            return False

        next_phase = MISSION_PHASE_TIMEOUT_TRANSITIONS.get(current_phase, Phase.PHASE6)
        print(
            f"{current_phase.name} cumulative timeout ({phase_elapsed:.1f}s / {phase_budget:.1f}s): "
            f"forcing {next_phase.name}"
        )
        self._commit_active_phase_elapsed(current_phase, now)
        if next_phase == Phase.PHASE6 and self.mission_end_reason == "RUNNING":
            self.mission_end_reason = f"{current_phase.name}_CUM_TIMEOUT_TO_PHASE6"
        self.initialize_phase(next_phase)
        return True

    def run(self, start_phase=Phase.PHASE0, allowed_phases=None):
        self.setup_hardware()
        self.signal_led(LED_SIGNAL_COUNT)
        self.initialize_phase(start_phase)
        allowed_set = None
        if allowed_phases is not None:
            allowed_set = {Phase(value) for value in allowed_phases}
        try:
            while not self._shutdown_requested:
                if allowed_set is not None:
                    current_phase = Phase(self.st.snapshot()["phase"])
                    if current_phase not in allowed_set:
                        print(f"Phase subset completed at phase {int(current_phase)}")
                        self.request_shutdown("PHASE_SUBSET_COMPLETED")
                        return
                self.loop_once()
                time.sleep(MAIN_LOOP_INTERVAL)
        except KeyboardInterrupt:
            print("\nKeyboardInterrupt")
            self.request_shutdown("KEYBOARD_INTERRUPT")
            print("Emergency stop requested. Motors are stopping.")
        finally:
            self.request_shutdown("RUN_EXIT")

    def loop_once(self):
        snapshot = self.st.snapshot()
        phase = Phase(snapshot["phase"])
        self._sync_phase_time_tracking(phase)
        if self._handle_timeout_transitions(phase):
            return
        self.led_blink_timer += 1
        handler = self.phase_handlers.get(phase)
        if handler is not None:
            handler.execute(self, snapshot)
            post_phase = Phase(self.st.snapshot()["phase"])
            self._sync_phase_time_tracking(post_phase)

    def _angle_diff_deg(self, target_deg, current_deg):
        diff = target_deg - current_deg
        if diff > 180.0:
            diff -= 360.0
        if diff < -180.0:
            diff += 360.0
        return diff

    def _normalize_heading_deg(self, value):
        try:
            deg = float(value)
        except (TypeError, ValueError):
            return None
        if not math.isfinite(deg):
            return None
        deg %= 360.0
        if deg < 0.0:
            deg += 360.0
        return deg

    def _update_bno_heading_offset_from_gps(self, snapshot):
        if not self._heading_offset_learning_ready(snapshot):
            return
        if not snapshot.get("angle_valid", False):
            return
        gps_heading = self._normalize_heading_deg(snapshot.get("gps_heading"))
        bno_heading = self._normalize_heading_deg(snapshot.get("angle"))
        if gps_heading is None or bno_heading is None:
            return

        observed_offset = self._angle_diff_deg(gps_heading, bno_heading)
        self._update_heading_offset_estimate("bno", observed_offset)

    def _update_mag_heading_offset_from_gps(self, snapshot, gps_heading, mag_heading):
        if not self._heading_offset_learning_ready(snapshot):
            return
        gps_heading = self._normalize_heading_deg(gps_heading)
        mag_heading = self._normalize_heading_deg(mag_heading)
        if gps_heading is None or mag_heading is None:
            return

        observed_offset = self._angle_diff_deg(gps_heading, mag_heading)
        self._update_heading_offset_estimate("mag", observed_offset)

    def _bno_heading_aligned_to_gps(self, snapshot):
        bno_heading = self._normalize_heading_deg(snapshot.get("angle"))
        if bno_heading is None:
            return None
        if not self.bno_heading_offset_valid:
            return bno_heading
        return self._normalize_heading_deg(bno_heading + self.bno_heading_offset_deg)

    def _mag_heading_aligned_to_gps(self, mag_heading):
        mag_heading = self._normalize_heading_deg(mag_heading)
        if mag_heading is None:
            return None
        if not self.mag_heading_offset_valid:
            return mag_heading
        return self._normalize_heading_deg(mag_heading + self.mag_heading_offset_deg)

    def _heading_offset_learning_ready(self, snapshot):
        if not snapshot.get("gps_heading_valid", False):
            return False
        speed = self._coerce_speed(snapshot.get("gps_speed_mps", 0.0))
        return speed >= float(HEADING_OFFSET_LEARN_MIN_SPEED_MPS)

    def _coerce_speed(self, value):
        try:
            speed = float(value)
        except (TypeError, ValueError):
            return 0.0
        if not math.isfinite(speed):
            return 0.0
        return max(0.0, speed)

    def _update_heading_offset_estimate(self, prefix, observed_offset):
        observed_offset = self._normalize_heading_deg(observed_offset)
        if observed_offset is None:
            return

        offset_attr = f"{prefix}_heading_offset_deg"
        valid_attr = f"{prefix}_heading_offset_valid"
        candidate_attr = f"{prefix}_heading_offset_candidate_deg"
        count_attr = f"{prefix}_heading_offset_candidate_count"
        residual_limit = max(1.0, float(HEADING_OFFSET_LEARN_MAX_RESIDUAL_DEG))

        if getattr(self, valid_attr, False):
            current_offset = self._normalize_heading_deg(getattr(self, offset_attr, 0.0))
            if current_offset is None:
                return
            residual = abs(self._angle_diff_deg(observed_offset, current_offset))
            if residual > residual_limit:
                return
            offset_delta = self._angle_diff_deg(observed_offset, current_offset)
            max_step = max(1.0, float(PHASE3_BNO_GPS_OFFSET_MAX_STEP_DEG))
            offset_delta = max(-max_step, min(max_step, offset_delta))
            alpha = max(0.0, min(1.0, float(PHASE3_BNO_GPS_OFFSET_ALPHA)))
            setattr(
                self,
                offset_attr,
                self._normalize_heading_deg(current_offset + offset_delta * alpha),
            )
            return

        candidate = self._normalize_heading_deg(getattr(self, candidate_attr, None))
        candidate_count = int(getattr(self, count_attr, 0))
        if candidate is None:
            setattr(self, candidate_attr, observed_offset)
            setattr(self, count_attr, 1)
            return

        residual = abs(self._angle_diff_deg(observed_offset, candidate))
        if residual > residual_limit:
            setattr(self, candidate_attr, observed_offset)
            setattr(self, count_attr, 1)
            return

        candidate_count += 1
        bootstrap_alpha = 1.0 / float(candidate_count)
        candidate = self._normalize_heading_deg(
            candidate + self._angle_diff_deg(observed_offset, candidate) * bootstrap_alpha
        )
        setattr(self, candidate_attr, candidate)
        setattr(self, count_attr, candidate_count)

        if candidate_count >= int(HEADING_OFFSET_LEARN_BOOTSTRAP_SAMPLES):
            setattr(self, offset_attr, candidate)
            setattr(self, valid_attr, True)

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
            aligned_bno = self._bno_heading_aligned_to_gps(snapshot)
            if aligned_bno is None:
                aligned_bno = self._normalize_heading_deg(snapshot.get("angle", 0.0))
            weights.append(bno_weight)
            headings.append(aligned_bno if aligned_bno is not None else 0.0)
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
