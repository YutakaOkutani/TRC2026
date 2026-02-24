import csv
import math
import os
import time
import traceback

import serial

from library import bno055
from library import detect_corn as dc

from cansat_mission.constants import (
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
    GPS_FIX_LOSS_TIMEOUT,
    GPS_HEADING_MIN_DIST,
    GPS_INACTIVE_DETECT,
    GPS_MAX_HDOP,
    GPS_MAX_SPEED_MPS,
    GPS_MIN_FIX_QUAL,
    GPS_MIN_SATELLITES,
    GPS_RECONNECT_SLEEP,
    GPS_STABLE_FIX_COUNT,
    PHASES_CAMERA_ACTIVE,
    SONAR_MAX_DISTANCE,
)
from cansat_mission.gps_utils import coerce_gga_metrics, gga_quality_ok, open_gps_serial, parse_gga_sentence
from cansat_mission.navigation import calc_distance_and_azimuth, current_milli_time


class SensorManager:
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

    def _angle_jump_ok(self, angle):
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

    def _try_reinit_camera(self):
        now = time.time()
        if now - self.camera_last_reinit < CAMERA_REINIT_INTERVAL:
            return
        self.camera_last_reinit = now
        try:
            detector = dc.detector()
            detector.set_roi_img(self.roi_img)
            self.devices[DEVICE_DETECTOR] = detector
            self.camera_fail_count = 0
            self.camera_dead_since = None
            print("Camera: Reinitialized.")
        except Exception as exc:
            print(f"Camera: Reinit error {exc}.")

    def get_bno_data(self):
        bno_instance = self.devices.get(DEVICE_BNO)
        if bno_instance is None:
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
                freeze = (
                    self._vector_near_zero(acc["value"], BNO_FREEZE_EPS)
                    and self._vector_near_zero(gyro["value"], BNO_FREEZE_EPS)
                    and self._vector_near_zero(mag["value"], BNO_FREEZE_EPS)
                    and euler_zero
                )
                if freeze:
                    self.bno_fail_count += 1
                    if self.bno_fail_count >= BNO_FAIL_LIMIT:
                        self._try_reinit_bno()

            acc_ok = (not freeze) and acc["valid"] and self._vector_within(acc["value"], BNO_ACC_MAX)
            gyro_ok = (not freeze) and gyro["valid"] and self._vector_within(gyro["value"], BNO_GYRO_MAX)
            mag_ok = (not freeze) and mag["valid"] and self._vector_within(mag["value"], BNO_MAG_MAX)

            angle_val = 0.0
            if euler["valid"] and len(euler["value"]) >= 1:
                angle_val = float(euler["value"][0])
            angle_ok = (
                (not freeze)
                and euler["valid"]
                and math.isfinite(angle_val)
                and 0.0 <= angle_val < 360.0
                and self._angle_jump_ok(angle_val)
            )

            sys_ok = sys_status["valid"] and sys_error["valid"]
            sys_error_ok = sys_ok and sys_error["value"] == 0
            fusion_ok = sys_ok and sys_status["value"] in BNO_FUSION_OK_STATES

            if acc_ok:
                self.bno_last_valid["acc"] = list(acc["value"])
            if gyro_ok:
                self.bno_last_valid["gyro"] = list(gyro["value"])
            if mag_ok:
                self.bno_last_valid["mag"] = list(mag["value"])
            if angle_ok:
                self.bno_last_valid["angle"] = angle_val
                self.bno_last_valid_time = time.time()

            acc_val = list(self.bno_last_valid["acc"])
            gyro_val = list(self.bno_last_valid["gyro"])
            mag_val = list(self.bno_last_valid["mag"])
            angle_val = float(self.bno_last_valid["angle"])
            fall = math.sqrt(acc_val[0] ** 2 + acc_val[1] ** 2 + acc_val[2] ** 2)

            calib_ok = calib["valid"] and calib["value"][3] >= BNO_CALIB_MAG_MIN
            angle_valid = angle_ok and calib_ok and sys_error_ok and fusion_ok
            self.bno_calib = calib if calib else DEFAULT_BNO_CALIB
            now = time.time()
            if self.bno_last_valid_time > 0:
                self.bno_stale_sec = now - self.bno_last_valid_time
            else:
                self.bno_stale_sec = 0.0
            if self.bno_stale_sec > BNO_STALE_TIMEOUT:
                angle_valid = False

            return {
                "acc": acc_val,
                "gyro": gyro_val,
                "mag": mag_val,
                "fall": fall,
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
            return None

    def get_bmp_data(self):
        bmp_instance = self.devices.get(DEVICE_BMP)
        if bmp_instance is None:
            return None
        try:
            return {"alt": bmp_instance.getAltitude(), "pres": bmp_instance.getPressure()}
        except Exception:
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
            self.state.update_cone(
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
                cdir = 1.0 - detector.cone_direction
            cone_method = str(getattr(detector, "debug_method", "unknown"))
            self.state.update_cone(
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
            self.state.update_cone(
                cone_direction=CONE_CENTER_POSITION,
                cone_probability=0.0,
                cone_is_reached=False,
                cone_method="camera_error",
            )

    def gps_thread(self):
        serial_obj, _, _ = open_gps_serial()
        last_buffer_clear = time.time()
        last_fix_time = 0.0
        last_valid_fix_time = 0.0
        last_valid_latlng = None
        stable_count = 0
        while True:
            try:
                if serial_obj is None or not serial_obj.is_open:
                    serial_obj, _, _ = open_gps_serial()
                    time.sleep(GPS_RECONNECT_SLEEP)
                    continue
                now = time.time()
                if last_valid_fix_time > 0 and now - last_valid_fix_time > GPS_FIX_LOSS_TIMEOUT:
                    stable_count = 0
                    self.state.update_gps(gps_detect=GPS_INACTIVE_DETECT, gps_heading_valid=False)
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
                    continue
                line = line_bytes.decode("utf-8", errors="ignore").strip()
                parsed = parse_gga_sentence(line)
                if parsed is None:
                    continue
                lat = parsed["lat"]
                lng = parsed["lng"]
                gps_qual = parsed["gps_qual"]
                num_sats = parsed["num_sats"]
                hdop = parsed["hdop"]
                qual_ok, sats_ok, hdop_ok = gga_quality_ok(gps_qual, num_sats, hdop)
                gps_fix_qual_val, gps_sats_val, gps_hdop_val = coerce_gga_metrics(gps_qual, num_sats, hdop)
                self.state.update_gps(
                    gps_fix_qual=gps_fix_qual_val,
                    gps_sats=gps_sats_val,
                    gps_hdop=gps_hdop_val,
                )

                if not (qual_ok and sats_ok and hdop_ok and (lat != 0.0 or lng != 0.0)):
                    stable_count = 0
                    continue

                speed_ok = True
                if last_valid_latlng is not None:
                    dist, _ = calc_distance_and_azimuth(last_valid_latlng[0], last_valid_latlng[1], lat, lng)
                    dt = now - last_fix_time if last_fix_time > 0 else 0
                    if dt > 0:
                        speed = dist / dt
                        if speed > GPS_MAX_SPEED_MPS:
                            speed_ok = False
                if not speed_ok:
                    stable_count = 0
                    continue

                stable_count += 1
                last_fix_time = now
                if stable_count >= GPS_STABLE_FIX_COUNT:
                    gps_heading = None
                    gps_heading_valid = False
                    if last_valid_latlng is not None:
                        dist, course = calc_distance_and_azimuth(last_valid_latlng[0], last_valid_latlng[1], lat, lng)
                        if dist >= GPS_HEADING_MIN_DIST:
                            gps_heading = course
                            gps_heading_valid = True
                    self.state.update_gps(
                        lat=lat,
                        lng=lng,
                        gps_detect=GPS_ACTIVE_DETECT,
                        gps_heading=gps_heading,
                        gps_heading_valid=gps_heading_valid,
                        gps_fix_qual=gps_fix_qual_val,
                        gps_sats=gps_sats_val,
                        gps_hdop=gps_hdop_val,
                    )
                    last_valid_fix_time = now
                    last_valid_latlng = (lat, lng)
                else:
                    self.state.update_gps(gps_detect=GPS_INACTIVE_DETECT, gps_heading_valid=False)
            except Exception:
                try:
                    if serial_obj is not None:
                        serial_obj.close()
                except Exception:
                    pass
                serial_obj = None
                print("GPS serial error; attempting reconnect.")
                time.sleep(GPS_RECONNECT_SLEEP)

    def camera_thread(self):
        while True:
            current_phase = self.state.snapshot()["phase"]
            if current_phase in PHASES_CAMERA_ACTIVE:
                self.cone_detect()
                time.sleep(CAMERA_ACTIVE_SLEEP)
            else:
                time.sleep(CAMERA_IDLE_SLEEP)

    def data_thread(self):
        while True:
            try:
                # Log the latest available snapshot first so abrupt termination leaves a row.
                current_data = self.state.snapshot()
                motor_cmd = getattr(self, "last_motor_command", {})
                mission_start = getattr(self, "mission_start_time", None)
                mission_elapsed_sec = 0.0
                if mission_start:
                    mission_elapsed_sec = max(0.0, time.time() - mission_start)
                with open(self.log_path, "a", newline="") as file_obj:
                    writer = csv.writer(file_obj)
                    writer.writerow(
                        [
                            current_milli_time(),
                            current_data["phase"],
                            f"{current_data['acc'][0]:.2f}",
                            f"{current_data['acc'][1]:.2f}",
                            f"{current_data['acc'][2]:.2f}",
                            f"{current_data['gyro'][0]:.2f}",
                            f"{current_data['gyro'][1]:.2f}",
                            f"{current_data['gyro'][2]:.2f}",
                            f"{current_data['mag'][0]:.2f}",
                            f"{current_data['mag'][1]:.2f}",
                            f"{current_data['mag'][2]:.2f}",
                            f"{current_data['lat']:.6f}",
                            f"{current_data['lng']:.6f}",
                            int(current_data.get("gps_fix_qual", 0)),
                            int(current_data.get("gps_sats", 0)),
                            f"{current_data.get('gps_hdop', 0.0):.2f}",
                            f"{current_data['alt']:.2f}",
                            f"{current_data['pres']:.2f}",
                            f"{current_data['distance']:.2f}",
                            f"{current_data['azimuth']:.2f}",
                            f"{self.target_lat:.6f}",
                            f"{self.target_lng:.6f}",
                            f"{current_data['angle']:.2f}",
                            f"{current_data['direction']:.2f}",
                            f"{current_data['fall']:.2f}",
                            f"{current_data['cone_direction']:.2f}",
                            f"{current_data['cone_probability']:.2f}",
                            current_data.get("cone_method", ""),
                            f"{current_data['obstacle_dist']:.2f}",
                            int(current_data.get("angle_valid", False)),
                            f"{self.bno_stale_sec:.2f}",
                            motor_cmd.get("type", ""),
                            int(motor_cmd.get("updated_ms", 0)),
                            f"{float(motor_cmd.get('motor1_speed', 0.0)):.2f}",
                            int(bool(motor_cmd.get("motor1_forward", 1))),
                            f"{float(motor_cmd.get('motor2_speed', 0.0)):.2f}",
                            int(bool(motor_cmd.get("motor2_forward", 1))),
                            getattr(self, "mission_end_reason", "RUNNING"),
                            int(bool(getattr(self, "mission_total_timeout_triggered", False))),
                            f"{mission_elapsed_sec:.2f}",
                        ]
                    )
                    file_obj.flush()
                    # On Windows (especially cloud-synced folders like OneDrive),
                    # fsync per row can block for a long time and stall logging.
                    if os.name != "nt":
                        os.fsync(file_obj.fileno())
            except Exception as exc:
                print(f"Log Error: {exc}")

            try:
                bno_data = self.get_bno_data()
                bmp_data = self.get_bmp_data()
                sonar_dist = self.get_sonar_data()
                if bno_data:
                    self.state.update_imu(
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
                else:
                    self.state.update_imu(angle_valid=False)
                if bmp_data:
                    self.state.update_barometer(
                        alt=bmp_data["alt"],
                        pres=bmp_data["pres"],
                    )
                if sonar_dist is not None:
                    self.state.update_obstacle(obstacle_dist=sonar_dist)
            except Exception as exc:
                print(f"Data Thread Error: {exc}")
                traceback.print_exc()
            time.sleep(DATA_SAMPLING_RATE)
