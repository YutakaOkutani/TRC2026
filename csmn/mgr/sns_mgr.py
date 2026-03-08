import csv
import math
import os
import time
import traceback

import serial

from lib import bno055
from lib import detect_corn as dc

from csmn.const import (
    BMP_ALTITUDE_MAX_VALID,
    BMP_ALTITUDE_MIN_VALID,
    BMP_PRESSURE_MAX_VALID,
    BMP_PRESSURE_MIN_VALID,
    BMP_SEA_LEVEL_PRESSURE_PA,
    BNO_ACC_MAX,
    BNO_ANGLE_JUMP_MAX,
    BNO_CALIB_MAG_MIN,
    BNO_FAIL_LIMIT,
    BNO_FREEZE_EPS,
    BNO_FUSION_OK_STATES,
    BNO_GYRO_MAX,
    BNO_MAG_MAX,
    BNO_REINIT_COOLDOWN,
    BNO_STALE_TIMEOUT,
    CAMERA_ACTIVE_SLEEP,
    CAMERA_DEAD_TIMEOUT,
    CAMERA_FAIL_LIMIT,
    CAMERA_IDLE_SLEEP,
    CAMERA_REINIT_INTERVAL,
    CONE_CENTER_POSITION,
    DATA_SAMPLING_RATE,
    DEFAULT_BNO_CALIB,
    DEVICE_BMP,
    DEVICE_BNO,
    DEVICE_DETECTOR,
    DEVICE_SONAR,
    GPS_ACTIVE_DETECT,
    GPS_BUFFER_CLEAR_INTERVAL,
    GPS_BUFFER_CLEAR_THRESHOLD,
    GPS_DIAGNOSTIC_LOG_INTERVAL,
    GPS_FIX_LOSS_TIMEOUT,
    GPS_HEADING_BASELINE_MIN_DIST,
    GPS_HEADING_HOLD_SEC,
    GPS_HEADING_MIN_DIST,
    GPS_HEADING_WINDOW_SEC,
    GPS_INACTIVE_DETECT,
    GPS_MAX_HDOP,
    GPS_MAX_SPEED_MPS,
    GPS_MIN_FIX_QUAL,
    GPS_MIN_SATELLITES,
    GPS_NO_DATA_REOPEN_TIMEOUT,
    GPS_NON_GGA_REOPEN_TIMEOUT,
    GPS_RECONNECT_SLEEP,
    GPS_STABLE_FIX_COUNT,
    PHASES_CAMERA_ACTIVE,
    SONAR_MAX_DISTANCE,
)
from csmn.gps_util import coerce_gga_metrics, gga_quality_ok, open_gps_serial, parse_gga_sentence
from csmn.nav import calc_distance_and_azimuth


class SensorManager:
    def _transform_cone_direction_for_control(self, cone_direction):
        """Map detector X position into the rover control frame."""
        try:
            cdir = float(cone_direction)
        except (TypeError, ValueError):
            return CONE_CENTER_POSITION
        if not math.isfinite(cdir):
            return CONE_CENTER_POSITION

        if bool(getattr(self, "camera_control_invert_x", False)):
            cdir = 1.0 - cdir

        return max(0.0, min(1.0, cdir))

    def _coerce_float(self, value, default=0.0):
        try:
            casted = float(value)
            if not math.isfinite(casted):
                return float(default)
            return casted
        except (TypeError, ValueError):
            return float(default)

    def _coerce_int(self, value, default=0):
        try:
            return int(value)
        except (TypeError, ValueError):
            return int(default)

    def _snapshot_vec3(self, snapshot, key):
        value = snapshot.get(key, (0.0, 0.0, 0.0))
        if not isinstance(value, (list, tuple)) or len(value) < 3:
            return [0.0, 0.0, 0.0]
        return [
            self._coerce_float(value[0]),
            self._coerce_float(value[1]),
            self._coerce_float(value[2]),
        ]

    def _build_log_row(self):
        current_data = self.st.snapshot()
        motor_cmd = getattr(self, "last_motor_command", {})
        mission_start = getattr(self, "mission_start_time", None)
        mission_elapsed_sec = 0.0
        if mission_start:
            mission_elapsed_sec = max(0.0, time.time() - mission_start)

        acc = self._snapshot_vec3(current_data, "acc")
        gyro = self._snapshot_vec3(current_data, "gyro")
        mag = self._snapshot_vec3(current_data, "mag")

        motor_cmd_updated_ms = self._coerce_int(motor_cmd.get("updated_ms", 0))
        motor_cmd_updated_elapsed_sec = 0.0
        if mission_start and motor_cmd_updated_ms > 0:
            motor_cmd_updated_elapsed_sec = max(0.0, (motor_cmd_updated_ms / 1000.0) - mission_start)

        return [
            f"{mission_elapsed_sec:.2f}",
            self._coerce_int(current_data.get("phase", 0)),
            f"{acc[0]:.2f}",
            f"{acc[1]:.2f}",
            f"{acc[2]:.2f}",
            f"{gyro[0]:.2f}",
            f"{gyro[1]:.2f}",
            f"{gyro[2]:.2f}",
            f"{mag[0]:.2f}",
            f"{mag[1]:.2f}",
            f"{mag[2]:.2f}",
            f"{self._coerce_float(current_data.get('lat', 0.0)):.6f}",
            f"{self._coerce_float(current_data.get('lng', 0.0)):.6f}",
            f"{self._coerce_float(current_data.get('gps_speed_mps', 0.0)):.2f}",
            self._coerce_int(current_data.get("gps_fix_qual", 0)),
            self._coerce_int(current_data.get("gps_sats", 0)),
            f"{self._coerce_float(current_data.get('gps_hdop', 0.0)):.2f}",
            f"{self._coerce_float(current_data.get('alt', 0.0)):.2f}",
            f"{self._coerce_float(current_data.get('pres', 0.0)):.2f}",
            f"{self._coerce_float(current_data.get('distance', 0.0)):.2f}",
            f"{self._coerce_float(current_data.get('azimuth', 0.0)):.2f}",
            f"{self._coerce_float(getattr(self, 'target_lat', 0.0)):.6f}",
            f"{self._coerce_float(getattr(self, 'target_lng', 0.0)):.6f}",
            f"{self._coerce_float(current_data.get('angle', 0.0)):.2f}",
            f"{self._coerce_float(current_data.get('direction', 0.0)):.2f}",
            f"{self._coerce_float(current_data.get('fall', 0.0)):.2f}",
            f"{self._coerce_float(current_data.get('cone_direction', 0.0)):.2f}",
            f"{self._coerce_float(current_data.get('cone_probability', 0.0)):.2f}",
            str(current_data.get("cone_method", "")),
            f"{self._coerce_float(current_data.get('obstacle_dist', 0.0)):.2f}",
            self._coerce_int(bool(current_data.get("angle_valid", False))),
            f"{self._coerce_float(getattr(self, 'bno_stale_sec', 0.0)):.2f}",
            self._coerce_int(
                bool(
                    getattr(self, "bno_last_acc_time", 0.0) > 0.0
                    and getattr(self, "bno_acc_stale_sec", 0.0) <= BNO_STALE_TIMEOUT
                )
            ),
            f"{self._coerce_float(getattr(self, 'bno_acc_stale_sec', 0.0)):.2f}",
            self._coerce_int(
                bool(
                    getattr(self, "bmp_last_valid_time", 0.0) > 0.0
                    and getattr(self, "bmp_stale_sec", 0.0) <= BNO_STALE_TIMEOUT
                )
            ),
            f"{self._coerce_float(getattr(self, 'bmp_stale_sec', 0.0)):.2f}",
            str(motor_cmd.get("type", "")),
            f"{motor_cmd_updated_elapsed_sec:.2f}",
            f"{self._coerce_float(motor_cmd.get('motor1_speed', 0.0)):.2f}",
            self._coerce_int(bool(motor_cmd.get("motor1_forward", 1))),
            f"{self._coerce_float(motor_cmd.get('motor2_speed', 0.0)):.2f}",
            self._coerce_int(bool(motor_cmd.get("motor2_forward", 1))),
            str(getattr(self, "phase7_arrival_reason", "RUNNING")),
            str(getattr(self, "mission_end_reason", "RUNNING")),
            self._coerce_int(bool(getattr(self, "mission_total_timeout_triggered", False))),
            f"{self._coerce_float(mission_elapsed_sec):.2f}",
        ]

    def _append_log_row(self, writer, file_obj):
        writer.writerow(self._build_log_row())
        file_obj.flush()
        # On Windows (especially cloud-synced folders like OneDrive),
        # fsync per row can block for a long time and stall logging.
        if os.name != "nt":
            os.fsync(file_obj.fileno())

    def log_thread(self):
        log_error_last_print = 0.0
        while True:
            try:
                with open(self.log_path, "a", newline="") as file_obj:
                    writer = csv.writer(file_obj)
                    self._append_log_row(writer, file_obj)
            except Exception as exc:
                now = time.time()
                if now - log_error_last_print >= 1.0:
                    print(f"Log Error: {exc}")
                    traceback.print_exc()
                    log_error_last_print = now
            time.sleep(DATA_SAMPLING_RATE)

    def _vector_within(self, vec, max_abs):
        try:
            for value in vec:
                if not math.isfinite(value) or abs(value) > max_abs:
                    return False
        except Exception:
            return False
        return True

    def _vector_near_zero(self, vec, eps):
        try:
            for value in vec:
                if not math.isfinite(value) or abs(value) > eps:
                    return False
        except Exception:
            return False
        return True

    def _vector_finite(self, vec):
        try:
            values = [float(value) for value in vec]
        except Exception:
            return False
        return len(values) >= 3 and all(math.isfinite(value) for value in values)

    def _vector_norm(self, vec):
        try:
            values = [float(value) for value in vec]
        except Exception:
            return None
        if len(values) < 3 or not all(math.isfinite(value) for value in values[:3]):
            return None
        return math.sqrt(values[0] ** 2 + values[1] ** 2 + values[2] ** 2)

    def _scalar_within(self, value, min_value, max_value):
        try:
            scalar = float(value)
        except (TypeError, ValueError):
            return False
        if not math.isfinite(scalar):
            return False
        return min_value <= scalar <= max_value

    def _mark_bno_acc_stale(self):
        now = time.time()
        if getattr(self, "bno_last_acc_time", 0.0) > 0.0:
            self.bno_acc_stale_sec = max(0.0, now - self.bno_last_acc_time)
        else:
            self.bno_acc_stale_sec = BNO_STALE_TIMEOUT + 1.0

    def _mark_bmp_stale(self):
        now = time.time()
        if getattr(self, "bmp_last_valid_time", 0.0) > 0.0:
            self.bmp_stale_sec = max(0.0, now - self.bmp_last_valid_time)
        else:
            self.bmp_stale_sec = BNO_STALE_TIMEOUT + 1.0

    def _angle_jump_ok(self, angle):
        # Bootstrap the jump filter: before the first accepted sample (or after long
        # dropout), there is no reliable previous heading to compare against.
        if self.bno_last_valid_time <= 0:
            return True
        last = self.bno_last_valid.get("angle", 0.0)
        diff = abs(((angle - last + 180.0) % 360.0) - 180.0)
        return diff <= BNO_ANGLE_JUMP_MAX

    def _try_reinit_bno(self):
        now = time.time()
        if now - self.bno_last_reinit_time < BNO_REINIT_COOLDOWN:
            return
        self.bno_last_reinit_time = now
        try:
            bno = bno055.BNO055()
            if bno.setUp():
                self.devices[DEVICE_BNO] = bno
                self.bno_fail_count = 0
                print("BNO055: Reinitialized after failures.")
            else:
                print("BNO055: Reinit failed.")
        except Exception as exc:
            print(f"BNO055: Reinit error {exc}.")

    def _try_reinit_camera(self, force=False, reason=None):
        now = time.time()
        if (not force) and now - self.camera_last_reinit < CAMERA_REINIT_INTERVAL:
            return
        self.camera_last_reinit = now
        if reason:
            print(f"Camera: Reinit requested ({reason}).")
        try:
            if hasattr(self, "_release_camera_detector"):
                self._release_camera_detector()
            detector = dc.detector()
            roi_reference = getattr(self, "roi_references", None)
            if not roi_reference:
                roi_reference = getattr(self, "roi_img", None)
            detector.set_roi_img(roi_reference)
            self.devices[DEVICE_DETECTOR] = detector
            self.camera_fail_count = 0
            self.camera_dead_since = None
            print("Camera: Reinitialized.")
        except Exception as exc:
            print(f"Camera: Reinit error {exc}.")

    def get_bno_data(self):
        bno_instance = self.devices.get(DEVICE_BNO)
        if bno_instance is None:
            self._mark_bno_acc_stale()
            return None
        try:
            acc = bno_instance.getAcc()
            gyro = bno_instance.getGyro()
            mag = bno_instance.getMag()
            euler = bno_instance.getEuler()
            calib = bno_instance.getCalibrationStatus()
            sys_status = bno_instance.getSystemStatus()
            sys_error = bno_instance.getSystemError()

            i2c_ok = acc["valid"] and gyro["valid"] and mag["valid"] and euler["valid"]
            if not i2c_ok:
                self.bno_fail_count += 1
                if self.bno_fail_count >= BNO_FAIL_LIMIT:
                    self._try_reinit_bno()
            else:
                self.bno_fail_count = 0

            freeze = False
            if i2c_ok:
                euler_zero = False
                if euler["valid"] and len(euler["value"]) >= 1:
                    try:
                        euler_zero = abs(float(euler["value"][0])) <= BNO_FREEZE_EPS
                    except Exception:
                        euler_zero = False
                # A real BNO055 at rest still reports gravity on accel and a non-zero
                # magnetic field vector. Treat all-zero raw vectors as a dead sensor
                # even when a stale heading register contains a tiny non-zero value.
                raw_vectors_dead = (
                    self._vector_near_zero(acc["value"], BNO_FREEZE_EPS)
                    and self._vector_near_zero(mag["value"], BNO_FREEZE_EPS)
                    and self._vector_near_zero(gyro["value"], BNO_FREEZE_EPS)
                )
                freeze = (
                    raw_vectors_dead
                    and euler_zero
                )
                if raw_vectors_dead and not freeze:
                    freeze = True
                if freeze:
                    self.bno_fail_count += 1
                    if self.bno_fail_count % 25 == 0:
                        print(
                            "BNO zero-output detected: "
                            f"acc={acc['value']} gyro={gyro['value']} mag={mag['value']} "
                            f"sys={sys_status.get('value')} err={sys_error.get('value')}"
                        )
                    if self.bno_fail_count >= BNO_FAIL_LIMIT:
                        self._try_reinit_bno()

            acc_ok = (not freeze) and acc["valid"] and self._vector_within(acc["value"], BNO_ACC_MAX)
            gyro_ok = (not freeze) and gyro["valid"] and self._vector_within(gyro["value"], BNO_GYRO_MAX)
            mag_ok = (not freeze) and mag["valid"] and self._vector_within(mag["value"], BNO_MAG_MAX)

            angle_val = 0.0
            if euler["valid"] and len(euler["value"]) >= 1:
                angle_val = float(euler["value"][0])
            angle_jump_ok = self._angle_jump_ok(angle_val)
            angle_ok = (
                (not freeze)
                and euler["valid"]
                and math.isfinite(angle_val)
                and 0.0 <= angle_val < 360.0
                and angle_jump_ok
            )

            sys_ok = sys_status["valid"] and sys_error["valid"]
            sys_error_ok = sys_ok and sys_error["value"] == 0
            fusion_ok = sys_ok and sys_status["value"] in BNO_FUSION_OK_STATES

            if acc_ok:
                self.bno_last_valid["acc"] = list(acc["value"])
                self.bno_last_acc_time = time.time()
            if gyro_ok:
                self.bno_last_valid["gyro"] = list(gyro["value"])
            if mag_ok:
                self.bno_last_valid["mag"] = list(mag["value"])
            if angle_ok:
                self.bno_last_valid["angle"] = angle_val
                self.bno_last_valid_time = time.time()

            now = time.time()
            self._mark_bno_acc_stale()
            acc_val = list(self.bno_last_valid["acc"]) if self.bno_last_acc_time > 0.0 else None
            gyro_val = list(self.bno_last_valid["gyro"])
            raw_mag_available = (
                mag.get("valid", False)
                and self._vector_finite(mag.get("value", ()))
                and (not self._vector_near_zero(mag.get("value", ()), BNO_FREEZE_EPS))
            )
            # Phase3 legacy steering derives heading directly from mag XY.
            mag_val = list(mag["value"]) if raw_mag_available else list(self.bno_last_valid["mag"])
            angle_val = float(self.bno_last_valid["angle"])
            fall = None
            if acc_val is not None:
                fall = math.sqrt(acc_val[0] ** 2 + acc_val[1] ** 2 + acc_val[2] ** 2)

            calib_ok = calib["valid"] and calib["value"][3] >= BNO_CALIB_MAG_MIN
            # Allow heading use when Euler/system status is healthy even if magnetometer
            # calibration is still incomplete (common during short ground E2E runs).
            # Field tests can keep fusion status/calibration low for a while; use a
            # sane Euler heading if it passes local consistency checks.
            angle_valid = angle_ok
            self.bno_calib = calib if calib else DEFAULT_BNO_CALIB
            if self.bno_last_valid_time > 0:
                self.bno_stale_sec = now - self.bno_last_valid_time
            else:
                # No accepted heading sample has ever been observed in this run.
                # Mark as stale so logs clearly show "BNO unavailable" instead of 0.0.
                self.bno_stale_sec = BNO_STALE_TIMEOUT + 1.0
            if self.bno_stale_sec > BNO_STALE_TIMEOUT:
                angle_valid = False
            if not angle_valid:
                invalid_count = int(getattr(self, "_bno_heading_invalid_count", 0)) + 1
                self._bno_heading_invalid_count = invalid_count
                if invalid_count % 50 == 0:
                    print(
                        "BNO heading invalid: "
                        f"i2c_ok={int(i2c_ok)} freeze={int(freeze)} euler_valid={int(euler['valid'])} "
                        f"angle={angle_val:.2f} jump_ok={int(angle_jump_ok)} stale={self.bno_stale_sec:.2f}s"
                    )
            else:
                self._bno_heading_invalid_count = 0

            return {
                "acc": acc_val,
                "gyro": gyro_val,
                "mag": mag_val,
                "fall": fall,
                "acc_valid": self.bno_last_acc_time > 0.0 and self.bno_acc_stale_sec <= BNO_STALE_TIMEOUT,
                "angle": angle_val,
                "valid": acc_ok and gyro_ok and mag_ok,
                "angle_valid": angle_valid,
                "calib": calib,
                "sys_status": sys_status,
                "sys_error": sys_error,
                "stale_sec": self.bno_stale_sec,
            }
        except Exception:
            self.bno_fail_count += 1
            if self.bno_fail_count >= BNO_FAIL_LIMIT:
                self._try_reinit_bno()
            self._mark_bno_acc_stale()
            return None

    def get_bmp_data(self):
        bmp_instance = self.devices.get(DEVICE_BMP)
        if bmp_instance is None:
            self._mark_bmp_stale()
            return None
        try:
            pres = float(bmp_instance.getPressure())
            if not self._scalar_within(pres, BMP_PRESSURE_MIN_VALID, BMP_PRESSURE_MAX_VALID):
                self._mark_bmp_stale()
                return None

            alt = 44330.0 * (1.0 - math.pow(pres / BMP_SEA_LEVEL_PRESSURE_PA, 1.0 / 5.255))
            if not self._scalar_within(alt, BMP_ALTITUDE_MIN_VALID, BMP_ALTITUDE_MAX_VALID):
                self._mark_bmp_stale()
                return None

            self.bmp_last_valid_time = time.time()
            self.bmp_stale_sec = 0.0
            return {"alt": alt, "pres": pres, "valid": True, "stale_sec": 0.0}
        except Exception:
            self._mark_bmp_stale()
            return None

    def get_sonar_data(self):
        sonar_instance = self.devices.get(DEVICE_SONAR)
        if sonar_instance is None:
            return None
        try:
            dist_m = sonar_instance.distance
            if dist_m is not None and 0 < dist_m < SONAR_MAX_DISTANCE:
                return dist_m * 100.0
        except Exception:
            pass
        return None

    def cone_detect(self):
        detector = self.devices.get(DEVICE_DETECTOR)
        if detector is None:
            if self.camera_dead_since is None:
                self.camera_dead_since = time.time()
            self._try_reinit_camera()
            self.st.update_cone(
                cone_direction=CONE_CENTER_POSITION,
                cone_probability=0.0,
                cone_is_reached=False,
                cone_method="detector_unavailable",
            )
            return
        try:
            captured = detector.detect_cone()
            if not captured:
                raise RuntimeError("camera capture failed")
            prob = detector.probability if detector.probability else 0.0
            cdir = CONE_CENTER_POSITION
            if detector.cone_direction is not None:
                # detector.cone_direction is in camera image coordinates.
                # Convert to mission convention, then apply mount compensation.
                cdir = 1.0 - float(detector.cone_direction)
                cdir = self._transform_cone_direction_for_control(cdir)
            cone_method = str(getattr(detector, "debug_method", "unknown"))
            self.st.update_cone(
                cone_direction=cdir,
                cone_probability=prob,
                cone_is_reached=detector.is_reached,
                cone_method=cone_method,
            )
            last_method = getattr(self, "_last_logged_cone_method", None)
            if cone_method != last_method:
                print(f"Cone detector method: {cone_method}")
                self._last_logged_cone_method = cone_method
            self.camera_fail_count = 0
            self.camera_dead_since = None
        except Exception:
            self.camera_fail_count += 1
            if self.camera_fail_count >= CAMERA_FAIL_LIMIT:
                self.devices[DEVICE_DETECTOR] = None
                if self.camera_dead_since is None:
                    self.camera_dead_since = time.time()
            self.st.update_cone(
                cone_direction=CONE_CENTER_POSITION,
                cone_probability=0.0,
                cone_is_reached=False,
                cone_method="camera_error",
            )

    def gps_thread(self):
        serial_obj, selected_port, selected_baud = open_gps_serial()
        last_buffer_clear = time.time()
        serial_opened_at = last_buffer_clear
        last_serial_data_time = last_buffer_clear
        last_gga_time = 0.0
        last_fix_time = 0.0
        last_valid_fix_time = 0.0
        last_valid_latlng = None
        recent_valid_fixes = []
        last_heading = None
        last_heading_time = 0.0
        stable_count = 0
        diag_last_log = 0.0
        diag = {
            "status": "OPENING",
            "raw_lines": 0,
            "gga_lines": 0,
            "qual_fail": 0,
            "speed_fail": 0,
            "empty_reads": 0,
            "last_line": "",
            "last_fix_qual": 0,
            "last_sats": 0,
            "last_hdop": 0.0,
            "last_reopen_reason": "INIT",
            "reopen_count": 0,
        }

        def _set_gps_inactive_state():
            self.st.update_gps(
                gps_detect=GPS_INACTIVE_DETECT,
                gps_heading_valid=False,
                gps_speed_mps=0.0,
            )

        def _force_reopen(reason, now_ts):
            nonlocal serial_obj, stable_count, recent_valid_fixes, last_valid_latlng
            nonlocal last_fix_time, last_valid_fix_time, last_heading, last_heading_time
            nonlocal serial_opened_at, last_serial_data_time, last_gga_time
            try:
                if serial_obj is not None:
                    serial_obj.close()
            except Exception:
                pass
            serial_obj = None
            stable_count = 0
            recent_valid_fixes = []
            last_valid_latlng = None
            last_fix_time = 0.0
            last_valid_fix_time = 0.0
            last_heading = None
            last_heading_time = 0.0
            serial_opened_at = now_ts
            last_serial_data_time = now_ts
            last_gga_time = 0.0
            diag["status"] = "REOPENING"
            diag["last_reopen_reason"] = reason
            diag["reopen_count"] += 1
            _set_gps_inactive_state()

        while True:
            try:
                if serial_obj is None or not serial_obj.is_open:
                    diag["status"] = "REOPENING"
                    serial_obj, selected_port, selected_baud = open_gps_serial()
                    if serial_obj is not None and serial_obj.is_open:
                        serial_opened_at = time.time()
                        last_serial_data_time = serial_opened_at
                        last_gga_time = 0.0
                    time.sleep(GPS_RECONNECT_SLEEP)
                    continue
                now = time.time()
                if now - diag_last_log >= GPS_DIAGNOSTIC_LOG_INTERVAL:
                    no_data_for = max(0.0, now - last_serial_data_time)
                    no_gga_for = max(0.0, now - max(last_gga_time, serial_opened_at))
                    print(
                        "GPS diag: "
                        f"port={selected_port or 'none'} baud={selected_baud or 'none'} "
                        f"status={diag['status']} raw={diag['raw_lines']} gga={diag['gga_lines']} "
                        f"qual_fail={diag['qual_fail']} speed_fail={diag['speed_fail']} "
                        f"empty={diag['empty_reads']} stable={stable_count}/{GPS_STABLE_FIX_COUNT} "
                        f"fix={diag['last_fix_qual']} sats={diag['last_sats']} hdop={diag['last_hdop']:.2f} "
                        f"no_data={no_data_for:.1f}s no_gga={no_gga_for:.1f}s "
                        f"reopen={diag['reopen_count']}({diag['last_reopen_reason']}) "
                        f"last={diag['last_line'] or 'none'}"
                    )
                    diag_last_log = now

                no_data_elapsed = now - last_serial_data_time
                if no_data_elapsed >= GPS_NO_DATA_REOPEN_TIMEOUT:
                    print(
                        "GPS watchdog: no serial bytes "
                        f"for {no_data_elapsed:.1f}s on {selected_port or 'unknown'} @ "
                        f"{selected_baud or 'unknown'}; forcing reopen."
                    )
                    _force_reopen("NO_BYTES_TIMEOUT", now)
                    time.sleep(GPS_RECONNECT_SLEEP)
                    continue

                no_gga_elapsed = now - max(last_gga_time, serial_opened_at)
                if diag["raw_lines"] > 0 and no_gga_elapsed >= GPS_NON_GGA_REOPEN_TIMEOUT:
                    print(
                        "GPS watchdog: serial bytes are present but no valid GGA "
                        f"for {no_gga_elapsed:.1f}s on {selected_port or 'unknown'} @ "
                        f"{selected_baud or 'unknown'}; forcing reopen."
                    )
                    _force_reopen("NO_GGA_TIMEOUT", now)
                    time.sleep(GPS_RECONNECT_SLEEP)
                    continue

                if last_valid_fix_time > 0 and now - last_valid_fix_time > GPS_FIX_LOSS_TIMEOUT:
                    stable_count = 0
                    recent_valid_fixes = []
                    diag["status"] = "FIX_LOST"
                    _set_gps_inactive_state()
                if serial_obj.in_waiting > GPS_BUFFER_CLEAR_THRESHOLD and now - last_buffer_clear >= GPS_BUFFER_CLEAR_INTERVAL:
                    try:
                        serial_obj.reset_input_buffer()
                        print("GPS buffer cleared to drop stale data.")
                    except Exception:
                        pass
                    last_buffer_clear = now
                    stable_count = 0
                line_bytes = serial_obj.readline()
                if not line_bytes:
                    diag["status"] = "NO_BYTES"
                    diag["empty_reads"] += 1
                    continue
                last_serial_data_time = now
                line = line_bytes.decode("utf-8", errors="ignore").strip()
                diag["raw_lines"] += 1
                diag["last_line"] = line[:120]
                parsed = parse_gga_sentence(line)
                if parsed is None:
                    diag["status"] = "NON_GGA"
                    continue
                last_gga_time = now
                diag["gga_lines"] += 1
                lat = parsed["lat"]
                lng = parsed["lng"]
                gps_qual = parsed["gps_qual"]
                num_sats = parsed["num_sats"]
                hdop = parsed["hdop"]
                # Record the latest parsed coordinates for logging/diagnostics even
                # before they satisfy the "stable fix" gating used for navigation.
                if lat != 0.0 or lng != 0.0:
                    self.st.update_gps(lat=lat, lng=lng)
                qual_ok, sats_ok, hdop_ok = gga_quality_ok(gps_qual, num_sats, hdop)
                gps_fix_qual_val, gps_sats_val, gps_hdop_val = coerce_gga_metrics(gps_qual, num_sats, hdop)
                diag["last_fix_qual"] = gps_fix_qual_val
                diag["last_sats"] = gps_sats_val
                diag["last_hdop"] = gps_hdop_val
                self.st.update_gps(
                    gps_fix_qual=gps_fix_qual_val,
                    gps_sats=gps_sats_val,
                    gps_hdop=gps_hdop_val,
                )

                if not (qual_ok and sats_ok and hdop_ok and (lat != 0.0 or lng != 0.0)):
                    stable_count = 0
                    diag["status"] = "GGA_REJECTED"
                    diag["qual_fail"] += 1
                    self.st.update_gps(gps_speed_mps=0.0)
                    continue

                speed_ok = True
                speed = 0.0
                if last_valid_latlng is not None:
                    dist, _ = calc_distance_and_azimuth(last_valid_latlng[0], last_valid_latlng[1], lat, lng)
                    dt = now - last_fix_time if last_fix_time > 0 else 0
                    if dt > 0:
                        speed = dist / dt
                        if speed > GPS_MAX_SPEED_MPS:
                            speed_ok = False
                if not speed_ok:
                    stable_count = 0
                    diag["status"] = "SPEED_REJECTED"
                    diag["speed_fail"] += 1
                    self.st.update_gps(gps_speed_mps=0.0)
                    continue

                stable_count += 1
                diag["status"] = "STABILIZING"
                last_fix_time = now
                if stable_count >= GPS_STABLE_FIX_COUNT:
                    gps_heading = None
                    gps_heading_valid = False
                    gps_speed_mps = 0.0
                    recent_valid_fixes.append((now, lat, lng))
                    cutoff = now - max(1.0, float(GPS_HEADING_WINDOW_SEC))
                    recent_valid_fixes = [fix for fix in recent_valid_fixes if fix[0] >= cutoff]
                    if last_valid_latlng is not None:
                        dist, course = calc_distance_and_azimuth(last_valid_latlng[0], last_valid_latlng[1], lat, lng)
                        dt_valid = now - last_valid_fix_time if last_valid_fix_time > 0 else 0.0
                        if dt_valid > 0:
                            gps_speed_mps = dist / dt_valid
                        if dist >= GPS_HEADING_MIN_DIST:
                            gps_heading = course
                            gps_heading_valid = True
                    if not gps_heading_valid and recent_valid_fixes:
                        best_dist = 0.0
                        best_course = None
                        for _, old_lat, old_lng in recent_valid_fixes:
                            span_dist, span_course = calc_distance_and_azimuth(old_lat, old_lng, lat, lng)
                            if span_dist >= GPS_HEADING_BASELINE_MIN_DIST and span_dist > best_dist:
                                best_dist = span_dist
                                best_course = span_course
                        if best_course is not None:
                            gps_heading = best_course
                            gps_heading_valid = True
                    if gps_heading_valid and gps_heading is not None:
                        last_heading = gps_heading
                        last_heading_time = now
                    elif last_heading is not None and (now - last_heading_time) <= GPS_HEADING_HOLD_SEC:
                        gps_heading = last_heading
                        gps_heading_valid = True
                    self.st.update_gps(
                        lat=lat,
                        lng=lng,
                        gps_detect=GPS_ACTIVE_DETECT,
                        gps_heading=gps_heading,
                        gps_heading_valid=gps_heading_valid,
                        gps_speed_mps=gps_speed_mps,
                        gps_fix_qual=gps_fix_qual_val,
                        gps_sats=gps_sats_val,
                        gps_hdop=gps_hdop_val,
                    )
                    diag["status"] = "ACTIVE"
                    last_valid_fix_time = now
                    last_valid_latlng = (lat, lng)
                else:
                    _set_gps_inactive_state()
            except Exception as exc:
                print(f"GPS serial error ({type(exc).__name__}: {exc}); attempting reconnect.")
                _force_reopen(f"SERIAL_ERROR:{type(exc).__name__}", time.time())
                time.sleep(GPS_RECONNECT_SLEEP)

    def camera_thread(self):
        while True:
            current_phase = self.st.snapshot()["phase"]
            if current_phase in PHASES_CAMERA_ACTIVE:
                t_start = time.time()
                self.cone_detect()
                elapsed = time.time() - t_start
                remain = float(CAMERA_ACTIVE_SLEEP) - elapsed
                if remain > 0.0:
                    time.sleep(remain)
            else:
                time.sleep(CAMERA_IDLE_SLEEP)

    def data_thread(self):
        suspicious_bno_counter = 0
        while True:
            bno_data = None
            try:
                bno_data = self.get_bno_data()
            except Exception as exc:
                print(f"BNO Thread Slice Error: {exc}")
                traceback.print_exc()
            if bno_data:
                self.st.update_imu(
                    acc=bno_data["acc"],
                    gyro=bno_data["gyro"],
                    mag=bno_data["mag"],
                    fall=bno_data["fall"],
                    angle=bno_data["angle"],
                    angle_valid=bno_data["angle_valid"],
                )
                if bno_data.get("sys_status", {}).get("valid") and bno_data.get("sys_error", {}).get("valid"):
                    if (
                        bno_data["sys_error"]["value"] != 0
                        or bno_data["sys_status"]["value"] not in BNO_FUSION_OK_STATES
                    ):
                        print(
                            f"BNO status warn: sys={bno_data['sys_status']['value']} "
                            f"err={bno_data['sys_error']['value']}"
                        )
                acc_norm = self._vector_norm(bno_data.get("acc"))
                mag_norm = self._vector_norm(bno_data.get("mag"))
                if bno_data.get("acc_valid") and acc_norm is not None and mag_norm is not None:
                    if acc_norm < 1.0 or mag_norm < 1.0:
                        suspicious_bno_counter += 1
                        if suspicious_bno_counter % 25 == 0:
                            print(
                                "BNO suspicious sample: "
                                f"acc={bno_data.get('acc')} |acc|={acc_norm:.3f} "
                                f"mag={bno_data.get('mag')} |mag|={mag_norm:.3f} "
                                f"angle={float(bno_data.get('angle', 0.0)):.2f}"
                            )
                    else:
                        suspicious_bno_counter = 0
            else:
                self.st.update_imu(angle_valid=False)

            try:
                bmp_data = self.get_bmp_data()
            except Exception as exc:
                print(f"BMP Thread Slice Error: {exc}")
                traceback.print_exc()
                bmp_data = None
            if bmp_data:
                self.st.update_barometer(
                    alt=bmp_data["alt"],
                    pres=bmp_data["pres"],
                )

            try:
                sonar_dist = self.get_sonar_data()
            except Exception as exc:
                print(f"Sonar Thread Slice Error: {exc}")
                traceback.print_exc()
                sonar_dist = None
            if sonar_dist is not None:
                self.st.update_obstacle(obstacle_dist=sonar_dist)
            time.sleep(DATA_SAMPLING_RATE)
