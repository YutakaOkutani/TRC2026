import csv
import datetime
import math
import os
import pynmea2
import time
import traceback

import serial

from library import bno055
from library import bmp180

# --- GPS settings (identical to main.py) ---
GPS_SERIAL_PORT = "/dev/serial0"
GPS_BAUDRATE = 115200  # 9600, 38400
GPS_SERIAL_TIMEOUT = 1.0
GPS_BUFFER_CLEAR_THRESHOLD = 2048  # bytes; flush when backlog grows too large
GPS_BUFFER_CLEAR_INTERVAL = 5.0    # seconds between flush attempts
GPS_MIN_FIX_QUAL = 1              # 1: GPS fix, 2: DGPS, 4/5: RTK
GPS_MIN_SATELLITES = 4
GPS_MAX_HDOP = 5.0
GPS_MAX_SPEED_MPS = 50.0          # reject if jump implies speed over this (m/s)
GPS_STABLE_FIX_COUNT = 3          # consecutive good fixes required
GPS_FIX_LOSS_TIMEOUT = 8.0        # seconds until detect flag drops when no valid fix

# --- Logging settings (mirrors main.py) ---
LOG_DIR = "testlog/landing_impact"
LOG_PREFIX = "robust_log_"
DATA_SAMPLING_RATE = 0.06
BNO_STALE_TIMEOUT = 2.0


def calc_distance_and_azimuth(lat1, lng1, lat2, lng2):
    """Great-circle distance (m) and azimuth (deg) identical to main.py."""
    R = 6378137.0
    rad_lat1 = math.radians(lat1)
    rad_lng1 = math.radians(lng1)
    rad_lat2 = math.radians(lat2)
    rad_lng2 = math.radians(lng2)
    d_lng = rad_lng2 - rad_lng1
    sin_lat1 = math.sin(rad_lat1)
    cos_lat1 = math.cos(rad_lat1)
    sin_lat2 = math.sin(rad_lat2)
    cos_lat2 = math.cos(rad_lat2)
    cos_d_lng = math.cos(d_lng)
    val = sin_lat1 * sin_lat2 + cos_lat1 * cos_lat2 * cos_d_lng
    val = max(-1.0, min(1.0, val))
    central_angle = math.acos(val)
    dist = R * central_angle
    y = math.sin(d_lng) * cos_lat2
    x = cos_lat1 * sin_lat2 - sin_lat1 * cos_lat2 * cos_d_lng
    azi = math.degrees(math.atan2(y, x))
    if azi < 0:
        azi += 360.0
    return dist, azi


def current_milli_time():
    return round(time.time() * 1000)


class RobustGPSReader:
    """GPS reader that mirrors main.py's parsing, gating, and stability checks."""

    def __init__(self):
        self.ser = None
        self.last_buffer_clear = time.time()
        self.last_fix_time = 0.0
        self.last_valid_fix_time = 0.0
        self.last_valid_latlng = None
        self.stable_count = 0

    def open_serial(self):
        try:
            ser = serial.Serial(GPS_SERIAL_PORT, GPS_BAUDRATE, timeout=GPS_SERIAL_TIMEOUT)
            try:
                ser.reset_input_buffer()
            except Exception:
                pass
            print("GPS serial ready.")
            return ser
        except Exception as exc:
            print(f"Could not open GPS serial port {GPS_SERIAL_PORT}: {exc}")
            return None

    def ensure_serial(self):
        if self.ser is None or not self.ser.is_open:
            self.ser = self.open_serial()

    def read_fix(self):
        """Return a stable fix dict or None using the same logic as main.py."""
        self.ensure_serial()
        if self.ser is None:
            time.sleep(1)
            return None

        now = time.time()
        if self.last_valid_fix_time > 0 and now - self.last_valid_fix_time > GPS_FIX_LOSS_TIMEOUT:
            self.stable_count = 0

        if (
            self.ser.in_waiting > GPS_BUFFER_CLEAR_THRESHOLD
            and now - self.last_buffer_clear >= GPS_BUFFER_CLEAR_INTERVAL
        ):
            try:
                self.ser.reset_input_buffer()
                print("GPS buffer cleared to drop stale data.")
            except Exception:
                pass
            self.last_buffer_clear = now
            self.stable_count = 0

        try:
            line_bytes = self.ser.readline()
        except Exception as exc:
            print(f"[GPS] Failed to read line: {exc}")
            try:
                if self.ser is not None:
                    self.ser.close()
            except Exception:
                pass
            self.ser = None
            time.sleep(1)
            return None

        if not line_bytes:
            return None

        line = line_bytes.decode("utf-8", errors="ignore").strip()
        if not (line.startswith("$GPGGA") or line.startswith("$GNGGA")):
            return None

        try:
            msg = pynmea2.parse(line, check=True)
        except Exception:
            return None

        if getattr(msg, "sentence_type", "") != "GGA":
            return None

        lat_val = getattr(msg, "latitude", None)
        lng_val = getattr(msg, "longitude", None)
        if lat_val is None or lng_val is None:
            return None

        lat = float(lat_val)
        lng = float(lng_val)

        gps_qual = getattr(msg, "gps_qual", None)
        num_sats = getattr(msg, "num_sats", None)
        hdop = getattr(msg, "horizontal_dil", None)

        try:
            qual_ok = gps_qual is not None and int(gps_qual) >= GPS_MIN_FIX_QUAL
        except (TypeError, ValueError):
            qual_ok = False
        try:
            sats_ok = num_sats is not None and int(num_sats) >= GPS_MIN_SATELLITES
        except (TypeError, ValueError):
            sats_ok = False
        try:
            hdop_ok = hdop is not None and float(hdop) <= GPS_MAX_HDOP
        except (TypeError, ValueError):
            hdop_ok = True

        if not (qual_ok and sats_ok and hdop_ok and (lat != 0.0 or lng != 0.0)):
            self.stable_count = 0
            return None

        speed_ok = True
        if self.last_valid_latlng is not None:
            dist, _ = calc_distance_and_azimuth(
                self.last_valid_latlng[0], self.last_valid_latlng[1], lat, lng
            )
            dt = now - self.last_fix_time if self.last_fix_time > 0 else 0
            if dt > 0:
                speed = dist / dt
                if speed > GPS_MAX_SPEED_MPS:
                    speed_ok = False

        if not speed_ok:
            self.stable_count = 0
            return None

        self.stable_count += 1
        self.last_fix_time = now

        if self.stable_count >= GPS_STABLE_FIX_COUNT:
            self.last_valid_fix_time = now
            self.last_valid_latlng = (lat, lng)
            return {
                "lat": lat,
                "lng": lng,
            }

        return None


class RobustBNOReader:
    """Minimal robust wrapper to provide angle_valid and stale age for logs."""

    def __init__(self, bno_instance):
        self.bno = bno_instance
        self.last_valid_time = 0.0
        self.last_angle = 0.0

    def read(self):
        if self.bno is None:
            return None
        try:
            acc = self.bno.getAcc()
            gyro = self.bno.getGyro()
            mag = self.bno.getMag()
            euler = self.bno.getEuler()
            calib = self.bno.getCalibrationStatus()
            sys_status = self.bno.getSystemStatus()
            sys_error = self.bno.getSystemError()

            acc_val = acc["value"]
            gyro_val = gyro["value"]
            mag_val = mag["value"]
            angle = euler["value"][0] if euler["valid"] and euler["value"] else 0.0
            fall = math.sqrt(acc_val[0] ** 2 + acc_val[1] ** 2 + acc_val[2] ** 2)

            calib_ok = calib["valid"] and calib["value"][3] >= 2
            sys_ok = sys_status["valid"] and sys_error["valid"]
            sys_error_ok = sys_ok and sys_error["value"] == 0
            fusion_ok = sys_ok and sys_status["value"] in (5, 6)

            angle_ok = euler["valid"] and math.isfinite(angle) and 0.0 <= angle < 360.0
            angle_valid = angle_ok and calib_ok and sys_error_ok and fusion_ok
            if angle_valid:
                self.last_angle = angle
                self.last_valid_time = time.time()

            stale_sec = 0.0
            if self.last_valid_time > 0:
                stale_sec = time.time() - self.last_valid_time
            if stale_sec > BNO_STALE_TIMEOUT:
                angle_valid = False

            return {
                "acc": acc_val,
                "gyro": gyro_val,
                "mag": mag_val,
                "fall": fall,
                "angle": self.last_angle,
                "angle_valid": angle_valid,
                "stale_sec": stale_sec,
            }
        except Exception:
            return None


def setup_sensors():
    """Initialize sensors and GPS serial connection."""
    bno = None
    bmp = None
    gps_reader = RobustGPSReader()

    print("Initializing BNO055...")
    try:
        bno = bno055.BNO055()
        if not bno.setUp():
            print("BNO055 setup returned False; disabling sensor.")
            bno = None
        else:
            print("BNO055 ready.")
    except Exception:
        print("Failed to initialize BNO055.")
        traceback.print_exc()
        bno = None

    print("Initializing BMP180...")
    try:
        bmp = bmp180.BMP180(oss=3)
        if not bmp.setUp():
            print("BMP180 setup returned False; disabling sensor.")
            bmp = None
        else:
            print("BMP180 ready.")
    except Exception:
        print("Failed to initialize BMP180.")
        traceback.print_exc()
        bmp = None

    print("Opening GPS serial port...")
    gps_reader.ensure_serial()

    return bno, bmp, gps_reader


def get_bmp_data(bmp_instance):
    """Return altitude and pressure identical to main.py."""
    if bmp_instance is None:
        return None
    try:
        return {"alt": bmp_instance.getAltitude(), "pres": bmp_instance.getPressure()}
    except Exception:
        return None


def init_log_file(log_path):
    """Write the identical CSV header main.py uses."""
    os.makedirs(LOG_DIR, exist_ok=True)
    with open(log_path, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow([
            "MilliTime", "Phase",
            "AccX", "AccY", "AccZ", "GyroX", "GyroY", "GyroZ", "MagX", "MagY", "MagZ",
            "LAT", "LNG", "ALT", "Pres",
            "Distance", "Azimuth", "Angle", "Direction", "Fall",
            "ConeDir", "ConeProb", "ObstacleDist",
            "AngleValid", "BNOStaleSec"
        ])


def main():
    bno, bmp, gps_reader = setup_sensors()
    bno_reader = RobustBNOReader(bno)

    if not any([bno, bmp, gps_reader and gps_reader.ser]):
        print("All sensors failed to initialize; exiting.")
        return

    now_time = datetime.datetime.now()
    log_path = os.path.join(LOG_DIR, LOG_PREFIX + now_time.strftime("%Y-%m%d-%H%M%S") + ".csv")
    try:
        init_log_file(log_path)
    except IOError as exc:
        print(f"Failed to create log file: {exc}")
        return

    last_lat = 0.0
    last_lng = 0.0
    try:
        with open(log_path, "a", newline="") as f:
            writer = csv.writer(f)
            while True:
                current_data = {
                    "phase": 0,
                    "acc": [0.0, 0.0, 0.0],
                    "gyro": [0.0, 0.0, 0.0],
                    "mag": [0.0, 0.0, 0.0],
                    "lat": last_lat,
                    "lng": last_lng,
                    "alt": 0.0,
                    "pres": 0.0,
                    "distance": 0.0,
                    "azimuth": 0.0,
                    "angle": 0.0,
                    "direction": 0.0,
                    "fall": 0.0,
                    "cone_direction": 0.0,
                    "cone_probability": 0.0,
                    "obstacle_dist": 0.0,
                    "angle_valid": 0,
                    "stale_sec": 0.0,
                }

                bno_data = bno_reader.read()
                bmp_data = get_bmp_data(bmp)
                gps_data = gps_reader.read_fix() if gps_reader else None

                if bno_data:
                    current_data.update({
                        "acc": bno_data["acc"],
                        "gyro": bno_data["gyro"],
                        "mag": bno_data["mag"],
                        "fall": bno_data["fall"],
                        "angle": bno_data["angle"],
                        "angle_valid": bno_data["angle_valid"],
                        "stale_sec": bno_data["stale_sec"],
                    })
                if bmp_data:
                    current_data.update({
                        "alt": bmp_data["alt"],
                        "pres": bmp_data["pres"],
                    })
                if gps_data:
                    last_lat = gps_data["lat"]
                    last_lng = gps_data["lng"]
                    current_data["lat"] = last_lat
                    current_data["lng"] = last_lng

                writer.writerow([
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
                    f"{current_data['alt']:.2f}",
                    f"{current_data['pres']:.2f}",
                    f"{current_data['distance']:.2f}",
                    f"{current_data['azimuth']:.2f}",
                    f"{current_data['angle']:.2f}",
                    f"{current_data['direction']:.2f}",
                    f"{current_data['fall']:.2f}",
                    f"{current_data['cone_direction']:.2f}",
                    f"{current_data['cone_probability']:.2f}",
                    f"{current_data['obstacle_dist']:.2f}",
                    int(current_data["angle_valid"]),
                    f"{current_data['stale_sec']:.2f}",
                ])
                f.flush()
                time.sleep(DATA_SAMPLING_RATE)
    except KeyboardInterrupt:
        print("\nMeasurement stopped by user.")
    except Exception:
        print("Unexpected error during logging.")
        traceback.print_exc()
    finally:
        if gps_reader and gps_reader.ser and gps_reader.ser.is_open:
            gps_reader.ser.close()
            print("Closed GPS serial port.")
        print(f"Log saved to {log_path}")


if __name__ == "__main__":
    main()
