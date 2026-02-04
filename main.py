import csv
import datetime
import math
import os
import pynmea2
import sys
import threading
import time

import cv2
import serial
from gpiozero import DigitalOutputDevice, DistanceSensor, LED, PWMOutputDevice
from gpiozero.pins.lgpio import LGPIOFactory
from picamera2 import Picamera2

from library import bmp180, bno055
from library import detect_corn as dc

# --- User Settings (tune here for debugging/operation) ---
# --- Logging ---
LOG_DIR = "./log"
LOG_PREFIX = "robust_log_"
# --- Target Location ---
# Latitude of target location
TARGET_LAT = 38.26052 
# Longitude of target location
TARGET_LNG = 140.8544151 
# --- Timeouts ---
TIMEOUT_PHASE_0 = 5 * 60
TIMEOUT_PHASE_1 = 30
TIMEOUT_PHASE_2 = 2 * 60
TIMEOUT_PHASE_3 = 5 * 60
TIMEOUT_PHASE_4 = 60
TIMEOUT_PHASE_5 = 45
DATA_SAMPLING_RATE = 0.06
# --- Impact Detection ---
IMPACT_FALL_THRESHOLD = 30.0
DROP_ALTITUDE_DIFF_THRESHOLD = 60.0
# --- Cone Detection ---
CONE_PROBABILITY_THRESHOLD = 0.1
CONE_LOST_COUNT_LIMIT = 10
# --- Parachute Separation ---
PARACHUTE_DIRECTION = -400.0
PARACHUTE_SEPARATION_SPEED = 100
PARACHUTE_MOTOR_PULSE = 0.05
# --- Calibration ---
CALIBRATION_TURN_SPEED = 50
CALIBRATION_MAG_THRESHOLD = 2
# --- Obstacle Avoidance ---
OBSTACLE_AVOID_DIST = 30.0
OBSTACLE_SPEED = 60
OBSTACLE_BACKUP_TIME = 1.0
OBSTACLE_TURN_TIME = 0.5
OBSTACLE_PAUSE_TIME = 0.2
# --- Cone Position ---
CONE_CENTER_POSITION = 0.5
# --- Camera Settings ---
CAMERA_ACTIVE_SLEEP = 0.05
CAMERA_IDLE_SLEEP = 0.5
CAMERA_REINIT_INTERVAL = 5.0
CAMERA_FAIL_LIMIT = 5
CAMERA_DEAD_TIMEOUT = 30.0
CAMERA_PHASE4_MAX_ATTEMPTS = 3
CAMERA_PHASE5_MAX_ATTEMPTS = 3
# --- File Paths ---
ROI_PATH_1 = os.path.join(LOG_DIR, "captured_roi_img.png")
ROI_PATH_2 = os.path.join(LOG_DIR, "captured.png")
# --- Hardware Settings ---
# --- Motor Settings ---
SEARCH_ROTATION_SPEED = 40
APPROACH_TURN_GAIN = 80
BASE_SPEED = 60
MOTOR_LOOP_INTERVAL = 0.05
MOTOR_RAMP_TIME = 0.6
MOTOR_RAMP_STEP = 0.05
# --- LED Settings ---
LED_INTERVAL_PHASE0 = 5
LED_INTERVAL_PHASE2 = 3
LED_INTERVAL_PHASE3 = 10
LED_INTERVAL_PHASE3_NEAR = 2
LED_INTERVAL_PHASE5 = 2
# Motor Pins ---
PIN_EN1 = 12
PIN_PH1 = 13
PIN_EN2 = 19
PIN_PH2 = 17
PWM_FREQ = 1000 # PWM Frequency in Hz
# LED Pins ---
PIN_LED_RED = 5
PIN_LED_GREEN = 6
# Sonar Pins ---
PIN_TRIG = 23
PIN_ECHO = 24
SONAR_MAX_DISTANCE = 4.0
# GPS Settings ---
GPS_SERIAL_PORT = "/dev/serial0"
GPS_BAUDRATE = 115200 # 9600, 38400
GPS_SERIAL_TIMEOUT = 1
GPS_HEADING_OFFSET = 5.43 # 種子島の磁気偏角（西偏）# Adjust if necessary
GPS_TURN_GAIN = 0.5
GPS_TURN_CLAMP = 30.0
GPS_CLOSE_DISTANCE = 5.0
GPS_BUFFER_CLEAR_THRESHOLD = 2048  # bytes; flush when backlog grows too large
GPS_BUFFER_CLEAR_INTERVAL = 5.0    # seconds between flush attempts
GPS_MIN_FIX_QUAL = 1              # 1: GPS fix, 2: DGPS, 4/5: RTK
GPS_MIN_SATELLITES = 4
GPS_MAX_HDOP = 5.0
GPS_MAX_SPEED_MPS = 50.0          # reject if jump implies speed over this (m/s)
GPS_STABLE_FIX_COUNT = 3          # consecutive good fixes required
GPS_FIX_LOSS_TIMEOUT = 8.0        # seconds until detect flag drops when no valid fix
GPS_HEADING_MIN_DIST = 1.5        # meters; minimum movement to trust GPS heading
# --- Other Constants ---
BNO_SETUP_RETRY_COUNT = 3
BNO_SETUP_RETRY_INTERVAL = 0.5
BNO_FAIL_LIMIT = 10
BNO_REINIT_COOLDOWN = 3.0
BNO_ACC_MAX = 200.0
BNO_GYRO_MAX = 2000.0
BNO_MAG_MAX = 2000.0
BNO_ANGLE_JUMP_MAX = 60.0
BNO_CALIB_MAG_MIN = 2
BNO_STALE_TIMEOUT = 2.0
BNO_FREEZE_EPS = 0.001

# --- Phase2 Calibration Motion ---
PHASE2_STRAIGHT_TIME = 6.0
PHASE2_FIG8_TIME = 25.0
PHASE2_TURN_INTERVAL = 3.0
PHASE2_SPEED = 45
PHASE2_TURN_BIAS = 20

# --- CanSat State Class ---
class CanSatState:
    # -- Initialization ---
    def __init__(self):
        self.lock = threading.Lock()
        self.acc = [0.0, 0.0, 0.0]
        self.gyro = [0.0, 0.0, 0.0]
        self.mag = [0.0, 0.0, 0.0]
        self.lat = 0.0
        self.lng = 0.0
        self.gps_heading = 0.0
        self.gps_heading_valid = False
        self.alt = 0.0
        self.pres = 0.0
        self.distance = 0.0
        self.azimuth = 0.0
        self.angle = 0.0
        self.angle_valid = False
        self.direction = 0.0
        self.fall = 0.0
        self.cone_direction = CONE_CENTER_POSITION
        self.cone_probability = 0.0
        self.obstacle_dist = 999.0
        self.phase = 0
        self.gps_detect = 0
        self.cone_is_reached = False

    # --- Update Methods ---
    def update_imu(self, acc=None, gyro=None, mag=None, fall=None, angle=None, angle_valid=None):
        with self.lock:
            if acc is not None:
                self.acc = acc
            if gyro is not None:
                self.gyro = gyro
            if mag is not None:
                self.mag = mag
            if fall is not None:
                self.fall = fall
            if angle is not None:
                self.angle = angle
            if angle_valid is not None:
                self.angle_valid = angle_valid
    def update_gps(self, lat=None, lng=None, gps_detect=None, gps_heading=None, gps_heading_valid=None):
        with self.lock:
            if lat is not None:
                self.lat = lat
            if lng is not None:
                self.lng = lng
            if gps_detect is not None:
                self.gps_detect = gps_detect
            if gps_heading is not None:
                self.gps_heading = gps_heading
            if gps_heading_valid is not None:
                self.gps_heading_valid = gps_heading_valid
    def update_barometer(self, alt=None, pres=None):
        with self.lock:
            if alt is not None:
                self.alt = alt
            if pres is not None:
                self.pres = pres
    def update_navigation(self, distance=None, azimuth=None, direction=None, phase=None):
        with self.lock:
            if distance is not None:
                self.distance = distance
            if azimuth is not None:
                self.azimuth = azimuth
            if direction is not None:
                self.direction = direction
            if phase is not None:
                self.phase = phase
    def update_cone(self, cone_direction=None, cone_probability=None, cone_is_reached=None):
        with self.lock:
            if cone_direction is not None:
                self.cone_direction = cone_direction
            if cone_probability is not None:
                self.cone_probability = cone_probability
            if cone_is_reached is not None:
                self.cone_is_reached = cone_is_reached
    def update_obstacle(self, obstacle_dist=None):
        with self.lock:
            if obstacle_dist is not None:
                self.obstacle_dist = obstacle_dist
    # --- Snapshot Method ---
    def snapshot(self):
        with self.lock:
            return {
                "acc": list(self.acc),
                "gyro": list(self.gyro),
                "mag": list(self.mag),
                "lat": self.lat,
                "lng": self.lng,
                "gps_heading": self.gps_heading,
                "gps_heading_valid": self.gps_heading_valid,
                "alt": self.alt,
                "pres": self.pres,
                "distance": self.distance,
                "azimuth": self.azimuth,
                "angle": self.angle,
                "angle_valid": self.angle_valid,
                "direction": self.direction,
                "fall": self.fall,
                "cone_direction": self.cone_direction,
                "cone_probability": self.cone_probability,
                "obstacle_dist": self.obstacle_dist,
                "phase": self.phase,
                "gps_detect": self.gps_detect,
                "cone_is_reached": self.cone_is_reached,
            }

# --- Utility Functions ---      
def current_milli_time():
    return round(time.time() * 1000)

# --- CanSat Controller Class ---
class CanSatController:

    # -- Initialization ---
    def __init__(self, target_lat, target_lng):
        self.state = CanSatState()
        self.target_lat = target_lat
        self.target_lng = target_lng
        now_time = datetime.datetime.now()
        os.makedirs(LOG_DIR, exist_ok=True)
        self.log_path = os.path.join(LOG_DIR, LOG_PREFIX + now_time.strftime("%Y-%m%d-%H%M%S") + ".csv")
        self.devices = {}
        self.led_blink_timer = 0
        self.searching_flag = False
        self.count_cone_lost = 0
        self.time_phase3_start = 0
        self.time_phase4_start = 0
        self.time_start_searching_cone = 0
        self.time_camera_start = 0
        self.motor_state = {}
        self.bno_fail_count = 0
        self.bno_last_reinit_time = 0.0
        self.bno_last_valid = {
            "acc": [0.0, 0.0, 0.0],
            "gyro": [0.0, 0.0, 0.0],
            "mag": [0.0, 0.0, 0.0],
            "angle": 0.0,
        }
        self.bno_last_valid_time = 0.0
        self.bno_stale_sec = 0.0
        self.bno_calib = {"valid": False, "value": (0, 0, 0, 0)}
        self.phase2_start_time = None
        self.phase2_stage = "straight"
        self.phase2_stage_start = None
        self.roi_img = None
        self.camera_fail_count = 0
        self.camera_last_reinit = 0.0
        self.camera_dead_since = None
        self.camera_phase4_attempts = 0
        self.camera_phase5_attempts = 0
        self.camera_phase4_start = None
        self.camera_phase5_start = None

    # --- Main entry ---
    def run(self):
        self.setup_hardware()
        self.signal_led(3)
        self.state.update_navigation(phase=0)
        try:
            while True:
                self.loop_once()
                time.sleep(0.1)
        except KeyboardInterrupt:
            print("\nKeyboardInterrupt")
            self.stop_motors()
            sys.exit()

    # --- Main phase loop ---
    def loop_once(self):
        st = self.state.snapshot()
        phase = st["phase"]
        self.led_blink_timer += 1
        if phase == 0:
            self.handle_phase0(st)
        elif phase == 1:
            self.handle_phase1()
        elif phase == 2:
            self.handle_phase2()
        elif phase == 3:
            self.handle_phase3(st)
        elif phase == 4:
            self.handle_phase4()
        elif phase == 5:
            self.handle_phase5()
        elif phase == 6:
            self.handle_phase6()

    # --- Phase handlers ---
    def handle_phase0(self, st):
        led_red = self.devices.get("led_red")
        led_green = self.devices.get("led_green")
        print("phase0 : falling")
        self.toggle_led(led_red, self.led_blink_timer, interval=LED_INTERVAL_PHASE0)
        if led_green:
            led_green.off()
        start = time.time()
        initial_alt = st["alt"]
        print(f"Start Altitude: {initial_alt:.2f}m")
        while True:
            self.led_blink_timer += 1
            self.toggle_led(led_red, self.led_blink_timer, interval=LED_INTERVAL_PHASE0)
            st_now = self.state.snapshot()
            is_impact = st_now["fall"] > IMPACT_FALL_THRESHOLD
            altitude_diff = initial_alt - st_now["alt"]
            is_drop = altitude_diff > DROP_ALTITUDE_DIFF_THRESHOLD
            if is_drop:
                print(f"Detected Drop: {altitude_diff:.2f}m")
                break
            if is_impact:
                print(f"Detected Impact: {st_now['fall']:.2f}m/s^2")
                break
            if time.time() - start > TIMEOUT_PHASE_0:
                print("Phase0 TIMEOUT: Force proceed (Sensor failure?)")
                break
            time.sleep(0.1)
        self.state.update_navigation(phase=1)
        self.time_phase1_start = time.time()

    def handle_phase1(self):
        # LED settings
        led_red = self.devices.get("led_red")
        led_green = self.devices.get("led_green")
        if led_red:
            led_red.on()
        if led_green:
            led_green.off()
        print("PH1: Start Parachute Separation")
        if not hasattr(self, "time_phase1_start") or self.time_phase1_start is None:
            self.time_phase1_start = time.time()
        elapsed = time.time() - self.time_phase1_start
        if elapsed < TIMEOUT_PHASE_1:
            self.state.update_navigation(direction=PARACHUTE_DIRECTION, phase=1)
            return  # Phase1 を継続
        print("PH1: Parachute Separation TIMEOUT → switching to Phase2")
        self.state.update_navigation(phase=2)
        self.phase2_start_time = time.time()
        self.phase2_stage = "straight"
        self.phase2_stage_start = self.phase2_start_time
        self.time_phase1_start = None

    def handle_phase2(self):
        led_red = self.devices.get("led_red")
        led_green = self.devices.get("led_green")
        print("phase2 : BNO Phase2 Calibration (Straight + Figure-8)")
        if led_red:
            led_red.off()
        if led_green:
            led_green.on()
        if self.phase2_start_time is None:
            self.phase2_start_time = time.time()
            self.phase2_stage = "straight"
            self.phase2_stage_start = self.phase2_start_time
        now = time.time()
        elapsed = now - self.phase2_start_time
        self.led_blink_timer += 1
        self.toggle_led(led_red, self.led_blink_timer, interval=LED_INTERVAL_PHASE2)

        if self.phase2_stage == "straight" and now - self.phase2_stage_start >= PHASE2_STRAIGHT_TIME:
            self.phase2_stage = "fig8"
            self.phase2_stage_start = now

        calib = self.bno_calib
        calib_ok = calib["valid"] and calib["value"][3] >= BNO_CALIB_MAG_MIN
        if self.led_blink_timer % 10 == 0 and calib["valid"]:
            sys_st, gyro_st, accel_st, mag_st = calib["value"]
            print(f"Calib Status: Sys={sys_st} Gyro={gyro_st} Acc={accel_st} Mag={mag_st}")

        if calib_ok:
            print("Calibration OK: Mag >= threshold")
            self.state.update_navigation(phase=3)
            self.time_phase3_start = now
            return
        if self.phase2_stage == "fig8" and now - self.phase2_stage_start >= PHASE2_FIG8_TIME:
            print("Phase2: Figure-8 complete → Phase3")
            self.state.update_navigation(phase=3)
            self.time_phase3_start = now
            return
        if elapsed > TIMEOUT_PHASE_2:
            print("Phase2 TIMEOUT: Force Phase 3 (Calibration Incomplete)")
            self.state.update_navigation(phase=3)
            self.time_phase3_start = now

    def handle_phase3(self, st):
        led_red = self.devices.get("led_red")
        led_green = self.devices.get("led_green")
        if led_red:
            led_red.off()
        self.toggle_led(led_green, self.led_blink_timer, interval=LED_INTERVAL_PHASE3)
        if time.time() - self.time_phase3_start > TIMEOUT_PHASE_3:
            print("Phase3 TIMEOUT: Give up GPS, switching to Camera")
            self.state.update_navigation(phase=4)
            self.time_phase4_start = time.time()
            return
        if st["gps_detect"] == 1:
            dist, azi = calc_distance_and_azimuth(st["lat"], st["lng"], self.target_lat, self.target_lng)
            self.state.update_navigation(distance=dist, azimuth=azi, direction=azi)
            if self.led_blink_timer % 10 == 0:
                if st["gps_heading_valid"]:
                    print(f"GPS Nav: Dist={dist:.1f}m, TargetDir={azi:.1f}, GPSHead={st['gps_heading']:.1f}")
                elif st["angle_valid"]:
                    print(f"GPS Nav: Dist={dist:.1f}m, TargetDir={azi:.1f}, BNOHead={st['angle']:.1f}")
                else:
                    print(f"GPS Nav: Dist={dist:.1f}m, TargetDir={azi:.1f}, MyHead=INVALID")
            if dist < GPS_CLOSE_DISTANCE:
                print(f"Close enough ({dist:.1f}m): Switching to Camera")
                self.state.update_navigation(phase=4)
            self.toggle_led(led_green, self.led_blink_timer, interval=LED_INTERVAL_PHASE3_NEAR)
        else:
            if self.led_blink_timer % 20 == 0:
                print("GPS Lost: Keep going...")

    def handle_phase4(self):
        led_red = self.devices.get("led_red")
        led_green = self.devices.get("led_green")
        st = self.state.snapshot()
        cone_prob = st["cone_probability"]
        print("phase4 : camera searching")
        if led_red:
            led_red.off()
        if led_green:
            led_green.on()
        if not self.searching_flag:
            self.searching_flag = True
            self.time_start_searching_cone = time.time()
            self.camera_phase4_attempts += 1
            self.camera_phase4_start = self.time_start_searching_cone
        else:
            if time.time() - self.time_start_searching_cone >= TIMEOUT_PHASE_4:
                print("Camera TIMEOUT: Cone not found or Camera dead")
                self.searching_flag = False
                self.state.update_navigation(phase=5)
                self.time_phase5_start = time.time()
        camera_dead = (
            self.camera_dead_since is not None
            and time.time() - self.camera_dead_since >= CAMERA_DEAD_TIMEOUT
        )
        if camera_dead and (
            self.camera_phase4_attempts >= CAMERA_PHASE4_MAX_ATTEMPTS
            or (self.camera_phase4_start is not None and time.time() - self.camera_phase4_start >= TIMEOUT_PHASE_4)
        ):
            print("Camera DEAD: Fallback to Phase3 (GPS/Straight)")
            fallback_dir = st["angle"] if st["angle_valid"] else st["direction"]
            self.state.update_navigation(direction=fallback_dir, phase=3)
            self.searching_flag = False
            self.time_phase3_start = time.time()
            return
        if cone_prob > CONE_PROBABILITY_THRESHOLD:
            self.state.update_navigation(phase=5)

    def handle_phase5(self):
        led_red = self.devices.get("led_red")
        led_green = self.devices.get("led_green")
        print("phase5 : approaching")
        self.time_camera_start = time.time()
        self.count_cone_lost = 0
        self.camera_phase5_attempts += 1
        self.camera_phase5_start = self.time_camera_start
        while True:
            self.led_blink_timer += 1
            if (self.led_blink_timer // LED_INTERVAL_PHASE5) % 2 == 0:
                if led_red:
                    led_red.on()
                if led_green:
                    led_green.off()
            else:
                if led_red:
                    led_red.off()
                if led_green:
                    led_green.on()
            st = self.state.snapshot()
            is_det = st["cone_probability"] > CONE_PROBABILITY_THRESHOLD
            is_reach = st["cone_is_reached"]
            camera_dead = (
                self.camera_dead_since is not None
                and time.time() - self.camera_dead_since >= CAMERA_DEAD_TIMEOUT
            )
            if camera_dead and (
                self.camera_phase5_attempts >= CAMERA_PHASE5_MAX_ATTEMPTS
                or (self.camera_phase5_start is not None and time.time() - self.camera_phase5_start >= TIMEOUT_PHASE_5)
            ):
                print("Camera DEAD: Fallback to Phase3 (GPS/Straight)")
                fallback_dir = st["angle"] if st["angle_valid"] else st["direction"]
                self.state.update_navigation(direction=fallback_dir, phase=3)
                break
            if not is_det:
                self.count_cone_lost += 1
            else:
                self.count_cone_lost = 0
            if self.count_cone_lost >= CONE_LOST_COUNT_LIMIT:
                self.state.update_navigation(phase=4)
                break
            if time.time() - self.time_camera_start >= TIMEOUT_PHASE_5:
                print("Phase5 TIMEOUT: Giving up, forcing Goal")
                self.state.update_navigation(phase=6)
                break
            if is_reach:
                print("Reached Cone! (Visual confirmation)")
                self.state.update_navigation(phase=6)
                break
            time.sleep(0.1)

    def handle_phase6(self):
        led_red = self.devices.get("led_red")
        led_green = self.devices.get("led_green")
        print("phase6 : Goal!!")
        if led_red:
            led_red.on()
        if led_green:
            led_green.on()
        self.stop_motors()
        sys.exit()

    # --- Hardware and threads ---
    def setup_hardware(self):
        print("--- Robust Setup Start ---")
        self.devices = {
            "bno": None,
            "bmp": None,
            "detector": None,
            "led_red": None,
            "led_green": None,
            "motor_1_pwm": None,
            "motor_1_dir": None,
            "motor_2_pwm": None,
            "motor_2_dir": None,
            "sonar": None,
        }
        try:
            bno = bno055.BNO055()
            for i in range(BNO_SETUP_RETRY_COUNT):
                if bno.setUp():
                    self.devices["bno"] = bno
                    print("BNO055: OK")
                    break
                print(f"BNO055: Retry {i+1}...")
                time.sleep(BNO_SETUP_RETRY_INTERVAL)
            else:
                print("WARNING: BNO055 Init Failed.")
        except Exception as e:
            print(f"BNO055: Critical Error {e}.")
        try:
            bmp = bmp180.BMP180(oss=3)
            if bmp.setUp():
                self.devices["bmp"] = bmp
                print("BMP180: OK")
            else:
                print("WARNING: BMP180 Init Failed.")
        except Exception as e:
            print(f"BMP180: Critical Error {e}.")
        print("Camera: Initializing...")
        try:
            detector = dc.detector()
            roi_img = None
            if os.path.exists(ROI_PATH_1):
                print(f"Loading ROI from {ROI_PATH_1}")
                roi_img = cv2.imread(ROI_PATH_1)
            elif os.path.exists(ROI_PATH_2):
                print(f"Loading ROI from {ROI_PATH_2}")
                roi_img = cv2.imread(ROI_PATH_2)
            else:
                print("WARNING: No ROI image found. Switching to DEFAULT RED detection.")
            self.roi_img = roi_img
            detector.set_roi_img(roi_img)
            detector.detect_cone()
            self.devices["detector"] = detector
            print("Camera: OK (Initialized)")
        except Exception as e:
            print(f"Camera: Critical Init Error {e}. Proceeding without Vision.")
            self.devices["detector"] = None
        print("GPIOZero: Initializing devices...")
        try:
            pin_factory = LGPIOFactory()
            self.devices["led_red"] = LED(PIN_LED_RED, pin_factory=pin_factory)
            self.devices["led_green"] = LED(PIN_LED_GREEN, pin_factory=pin_factory)
            self.devices["motor_1_pwm"] = PWMOutputDevice(PIN_EN1, pin_factory=pin_factory, frequency=PWM_FREQ, initial_value=0)
            self.devices["motor_1_dir"] = DigitalOutputDevice(PIN_PH1, pin_factory=pin_factory, initial_value=False)
            self.devices["motor_2_pwm"] = PWMOutputDevice(PIN_EN2, pin_factory=pin_factory, frequency=PWM_FREQ, initial_value=0)
            self.devices["motor_2_dir"] = DigitalOutputDevice(PIN_PH2, pin_factory=pin_factory, initial_value=False)
            self.devices["sonar"] = DistanceSensor(echo=PIN_ECHO, trigger=PIN_TRIG, max_distance=SONAR_MAX_DISTANCE, pin_factory=pin_factory)
            self.motor_state = {}
            for key in ("motor_1_pwm", "motor_2_pwm"):
                pwm_dev = self.devices.get(key)
                if pwm_dev:
                    self.motor_state[pwm_dev] = {"speed": 0.0, "direction": True}
            self.stop_motors()
            print("GPIOZero: OK")
        except Exception as e:
            print(f"GPIOZero Setup Error {e}.")
        self.start_threads()
        self.init_log_file()
        print("--- Setup Finished (Ready to Die Trying) ---")

    # --- Thread starters ---
    def start_threads(self):
        try:
            threading.Thread(target=self.move_motor_thread, daemon=True).start()
            threading.Thread(target=self.data_thread, daemon=True).start()
            threading.Thread(target=self.gps_thread, daemon=True).start()
            threading.Thread(target=self.camera_thread, daemon=True).start()
        except Exception as e:
            print(f"Thread Start Error {e}.")

    # --- Logging ---
    def init_log_file(self):
        try:
            with open(self.log_path, "w", newline="") as f:
                writer = csv.writer(f)
                writer.writerow([
                    "MilliTime", "Phase",
                    "AccX", "AccY", "AccZ", "GyroX", "GyroY", "GyroZ", "MagX", "MagY", "MagZ",
                    "LAT", "LNG", "ALT", "Pres",
                    "Distance", "Azimuth", "Angle", "Direction", "Fall",
                    "ConeDir", "ConeProb", "ObstacleDist",
                    "AngleValid", "BNOStaleSec"
                ])
        except Exception:
            print("Log File Init Failed. No logging.")

    # --- Sensor readers ---
    def _vector_within(self, vec, max_abs):
        try:
            for v in vec:
                if not math.isfinite(v) or abs(v) > max_abs:
                    return False
        except Exception:
            return False
        return True

    def _vector_near_zero(self, vec, eps):
        try:
            for v in vec:
                if not math.isfinite(v) or abs(v) > eps:
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
                self.devices["bno"] = bno
                self.bno_fail_count = 0
                print("BNO055: Reinitialized after failures.")
            else:
                print("BNO055: Reinit failed.")
        except Exception as e:
            print(f"BNO055: Reinit error {e}.")

    def _try_reinit_camera(self):
        now = time.time()
        if now - self.camera_last_reinit < CAMERA_REINIT_INTERVAL:
            return
        self.camera_last_reinit = now
        try:
            detector = dc.detector()
            detector.set_roi_img(self.roi_img)
            detector.detect_cone()
            self.devices["detector"] = detector
            self.camera_fail_count = 0
            self.camera_dead_since = None
            print("Camera: Reinitialized.")
        except Exception as e:
            print(f"Camera: Reinit error {e}.")

    def get_bno_data(self):
        bno_instance = self.devices.get("bno")
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
            fusion_ok = sys_ok and sys_status["value"] in (5, 6)

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
            self.bno_calib = calib
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
        bmp_instance = self.devices.get("bmp")
        if bmp_instance is None:
            return None
        try:
            return {"alt": bmp_instance.getAltitude(), "pres": bmp_instance.getPressure()}
        except Exception:
            return None
        
    def get_sonar_data(self):
        sonar_instance = self.devices.get("sonar")
        if sonar_instance is None:
            return None
        try:
            dist_m = sonar_instance.distance
            if dist_m is not None and 0 < dist_m < SONAR_MAX_DISTANCE:
                return dist_m * 100.0
        except Exception:
            pass
        return None
    # --- Cone detection ---
    def cone_detect(self):
        detector = self.devices.get("detector")
        if detector is None:
            if self.camera_dead_since is None:
                self.camera_dead_since = time.time()
            self._try_reinit_camera()
            self.state.update_cone(cone_direction=CONE_CENTER_POSITION, cone_probability=0.0, cone_is_reached=False)
            return
        try:
            detector.detect_cone()
            prob = detector.probability if detector.probability else 0.0
            cdir = CONE_CENTER_POSITION
            if detector.cone_direction is not None:
                cdir = 1.0 - detector.cone_direction
            self.state.update_cone(cone_direction=cdir, cone_probability=prob, cone_is_reached=detector.is_reached)
            self.camera_fail_count = 0
            self.camera_dead_since = None
        except Exception:
            self.camera_fail_count += 1
            if self.camera_fail_count >= CAMERA_FAIL_LIMIT:
                self.devices["detector"] = None
                if self.camera_dead_since is None:
                    self.camera_dead_since = time.time()
            self.state.update_cone(cone_direction=CONE_CENTER_POSITION, cone_probability=0.0, cone_is_reached=False)

    # --- Thread loops ---
    def gps_thread(self):
        def open_serial():
            try:
                ser = serial.Serial(GPS_SERIAL_PORT, GPS_BAUDRATE, timeout=GPS_SERIAL_TIMEOUT)
                try:
                    ser.reset_input_buffer()
                except Exception:
                    pass
                print("GPS serial opened.")
                return ser
            except Exception as e:
                print(f"GPS Serial Open Failed: {e}")
                return None

        s = open_serial()
        last_buffer_clear = time.time()
        last_fix_time = 0.0
        last_valid_fix_time = 0.0
        last_valid_latlng = None
        stable_count = 0
        while True:
            try:
                if s is None or not s.is_open:
                    s = open_serial()
                    time.sleep(1)
                    continue
                now = time.time()
                if last_valid_fix_time > 0 and now - last_valid_fix_time > GPS_FIX_LOSS_TIMEOUT:
                    self.state.update_gps(gps_detect=0, gps_heading_valid=False)
                if (
                    s.in_waiting > GPS_BUFFER_CLEAR_THRESHOLD
                    and now - last_buffer_clear >= GPS_BUFFER_CLEAR_INTERVAL
                ):
                    try:
                        s.reset_input_buffer()
                        print("GPS buffer cleared to drop stale data.")
                    except Exception:
                        pass
                    last_buffer_clear = now
                line_bytes = s.readline()
                if not line_bytes:
                    continue
                line = line_bytes.decode("utf-8", errors="ignore").strip()
                # Lightweight filter: only keep GGA sentences (contain fix/quality/hdop we need)
                if not (line.startswith("$GPGGA") or line.startswith("$GNGGA")):
                    continue
                try:
                    msg = pynmea2.parse(line, check=True)
                except Exception:
                    continue
                if getattr(msg, "sentence_type", "") != "GGA":
                    continue

                lat_val = getattr(msg, "latitude", None)
                lng_val = getattr(msg, "longitude", None)
                if lat_val is None or lng_val is None:
                    continue
                lat = float(lat_val)
                lng = float(lng_val)

                # Fix gate: quality / satellites / HDOP
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
                    hdop_ok = True  # allow if missing

                if not (qual_ok and sats_ok and hdop_ok and (lat != 0.0 or lng != 0.0)):
                    stable_count = 0
                    continue

                # Outlier/speed gate
                speed_ok = True
                if last_valid_latlng is not None:
                    dist, _ = calc_distance_and_azimuth(
                        last_valid_latlng[0], last_valid_latlng[1], lat, lng
                    )
                    dt = now - last_fix_time if last_fix_time > 0 else 0
                    if dt > 0:
                        speed = dist / dt
                        if speed > GPS_MAX_SPEED_MPS:
                            speed_ok = False
                if not speed_ok:
                    stable_count = 0
                    continue

                # Stability counter
                stable_count += 1
                last_fix_time = now
                if stable_count >= GPS_STABLE_FIX_COUNT:
                    gps_heading = None
                    gps_heading_valid = False
                    if last_valid_latlng is not None:
                        dist, course = calc_distance_and_azimuth(
                            last_valid_latlng[0], last_valid_latlng[1], lat, lng
                        )
                        if dist >= GPS_HEADING_MIN_DIST:
                            gps_heading = course
                            gps_heading_valid = True
                    self.state.update_gps(
                        lat=lat,
                        lng=lng,
                        gps_detect=1,
                        gps_heading=gps_heading,
                        gps_heading_valid=gps_heading_valid,
                    )
                    last_valid_fix_time = now
                    last_valid_latlng = (lat, lng)
                else:
                    self.state.update_gps(gps_detect=0, gps_heading_valid=False)
            except Exception:
                try:
                    if s is not None:
                        s.close()
                except Exception:
                    pass
                s = None
                print("GPS serial error; attempting reconnect.")
                time.sleep(1)

    def camera_thread(self):
        while True:
            current_phase = self.state.snapshot()["phase"]
            if current_phase in [4, 5]:
                self.cone_detect()
                time.sleep(CAMERA_ACTIVE_SLEEP)
            else:
                time.sleep(CAMERA_IDLE_SLEEP)

    def data_thread(self):
        while True:
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
                    if bno_data["sys_error"]["value"] != 0 or bno_data["sys_status"]["value"] not in (5, 6):
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
            current_data = self.state.snapshot()
            try:
                with open(self.log_path, "a", newline="") as f:
                    writer = csv.writer(f)
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
                        int(current_data.get("angle_valid", False)),
                        f"{self.bno_stale_sec:.2f}",
                    ])
            except Exception as e:
                print(f"Log Error: {e}")
            time.sleep(DATA_SAMPLING_RATE)

    def move_motor_thread(self):
        while True:
            st = self.state.snapshot()
            phase = st["phase"]
            obstacle_dist = st["obstacle_dist"]
            direction = st["direction"]
            angle = st["angle"]
            cone_direction = st["cone_direction"]
            if phase == 0 or phase == 6:
                self.stop_motors()
                time.sleep(0.1)
                continue
            if phase not in [0, 1, 5, 6] and obstacle_dist < OBSTACLE_AVOID_DIST:
                print(f"Obstacle Detected! {obstacle_dist:.1f}cm")
                self.stop_motors()
                time.sleep(OBSTACLE_PAUSE_TIME)
                self.set_motors(OBSTACLE_SPEED, False, OBSTACLE_SPEED, False)
                time.sleep(OBSTACLE_BACKUP_TIME)
                self.set_motors(OBSTACLE_SPEED, False, OBSTACLE_SPEED, True)
                time.sleep(OBSTACLE_TURN_TIME)
                self.stop_motors()
                time.sleep(OBSTACLE_PAUSE_TIME)
                continue
            if phase == 1 and direction == PARACHUTE_DIRECTION:
                self.set_motors(
                    PARACHUTE_SEPARATION_SPEED, True,
                    PARACHUTE_SEPARATION_SPEED, True,
                    ramp_time=0,
                )
                time.sleep(PARACHUTE_MOTOR_PULSE)
                continue
            if phase == 2:
                if self.phase2_stage == "straight":
                    self.set_motors(PHASE2_SPEED, True, PHASE2_SPEED, True)
                else:
                    elapsed = 0.0
                    if self.phase2_stage_start is not None:
                        elapsed = time.time() - self.phase2_stage_start
                    left_turn = int(elapsed // PHASE2_TURN_INTERVAL) % 2 == 0
                    bias = max(0, min(100, PHASE2_TURN_BIAS))
                    base = max(0, min(100, PHASE2_SPEED))
                    if left_turn:
                        speed_l = max(0, min(100, base - bias))
                        speed_r = max(0, min(100, base + bias))
                    else:
                        speed_l = max(0, min(100, base + bias))
                        speed_r = max(0, min(100, base - bias))
                    self.set_motors(speed_r, True, speed_l, True)
                time.sleep(MOTOR_LOOP_INTERVAL)
                continue
            if phase == 3:
                target_heading = direction
                if st.get("gps_heading_valid", False):
                    current_heading = st.get("gps_heading", 0.0)
                    diff = target_heading - current_heading
                    if diff > 180:
                        diff -= 360
                    if diff < -180:
                        diff += 360
                    turn_val = diff * GPS_TURN_GAIN
                    turn_val = max(-GPS_TURN_CLAMP, min(GPS_TURN_CLAMP, turn_val))
                    speed_l = max(0, min(100, BASE_SPEED + turn_val))
                    speed_r = max(0, min(100, BASE_SPEED - turn_val))
                    self.set_motors(speed_r, True, speed_l, True)
                elif st["angle_valid"]:
                    current_heading = angle
                    target_heading = direction + GPS_HEADING_OFFSET
                    diff = target_heading - current_heading
                    if diff > 180:
                        diff -= 360
                    if diff < -180:
                        diff += 360
                    turn_val = diff * GPS_TURN_GAIN
                    turn_val = max(-GPS_TURN_CLAMP, min(GPS_TURN_CLAMP, turn_val))
                    speed_l = max(0, min(100, BASE_SPEED + turn_val))
                    speed_r = max(0, min(100, BASE_SPEED - turn_val))
                    self.set_motors(speed_r, True, speed_l, True)
                else:
                    self.set_motors(BASE_SPEED, True, BASE_SPEED, True)
            elif phase == 4:
                self.set_motors(SEARCH_ROTATION_SPEED, True, SEARCH_ROTATION_SPEED, False)
            elif phase == 5:
                err = cone_direction - CONE_CENTER_POSITION
                turn_cam = err * APPROACH_TURN_GAIN
                speed_l = max(0, min(100, BASE_SPEED + turn_cam))
                speed_r = max(0, min(100, BASE_SPEED - turn_cam))
                self.set_motors(speed_r, True, speed_l, True)
            time.sleep(MOTOR_LOOP_INTERVAL)

    def _ramp_pwm(self, pwm_dev, start_speed, target_speed, ramp_time, step_interval=MOTOR_RAMP_STEP):
        """Ramp PWM duty in small steps to avoid sudden current draw."""
        if pwm_dev is None:
            return target_speed
        if ramp_time <= 0 or step_interval <= 0:
            pwm_dev.value = max(0.0, min(1.0, target_speed / 100.0))
            return target_speed
        steps = max(1, int(ramp_time / step_interval))
        step_duration = ramp_time / steps
        for step in range(1, steps + 1):
            duty = start_speed + (target_speed - start_speed) * (step / steps)
            pwm_dev.value = max(0.0, min(1.0, duty / 100.0))
            time.sleep(step_duration)
        return target_speed

    def _ramp_pwm_dual(
        self,
        pwm_a,
        start_a,
        target_a,
        pwm_b,
        start_b,
        target_b,
        ramp_time,
        step_interval=MOTOR_RAMP_STEP,
    ):
        """Ramp two PWM devices together so both motors start/stop in sync."""
        if pwm_a is None and pwm_b is None:
            return start_a, start_b
        if ramp_time <= 0 or step_interval <= 0:
            if pwm_a is not None:
                pwm_a.value = max(0.0, min(1.0, target_a / 100.0))
            if pwm_b is not None:
                pwm_b.value = max(0.0, min(1.0, target_b / 100.0))
            return target_a, target_b
        steps = max(1, int(ramp_time / step_interval))
        step_duration = ramp_time / steps
        for step in range(1, steps + 1):
            duty_a = start_a + (target_a - start_a) * (step / steps)
            duty_b = start_b + (target_b - start_b) * (step / steps)
            if pwm_a is not None:
                pwm_a.value = max(0.0, min(1.0, duty_a / 100.0))
            if pwm_b is not None:
                pwm_b.value = max(0.0, min(1.0, duty_b / 100.0))
            time.sleep(step_duration)
        return target_a, target_b

    def set_motor(self, motor_pwm, motor_dir, speed, forward, ramp_time=MOTOR_RAMP_TIME, step_interval=MOTOR_RAMP_STEP):
        if motor_pwm is None or motor_dir is None:
            return
        state = self.motor_state.setdefault(motor_pwm, {"speed": 0.0, "direction": True})
        current_speed = state["speed"]
        current_direction = state["direction"]

        # If direction changes, ramp to zero first to reduce stress on the driver.
        if current_speed > 0 and forward != current_direction:
            current_speed = self._ramp_pwm(motor_pwm, current_speed, 0, ramp_time / 2, step_interval)

        motor_dir.value = 1 if forward else 0
        target_speed = max(0.0, min(100.0, speed))
        current_speed = self._ramp_pwm(motor_pwm, current_speed, target_speed, ramp_time, step_interval)
        state["speed"] = current_speed
        state["direction"] = forward

    def set_motors(
        self,
        speed_a,
        forward_a,
        speed_b,
        forward_b,
        ramp_time=MOTOR_RAMP_TIME,
        step_interval=MOTOR_RAMP_STEP,
    ):
        motor_1_pwm = self.devices.get("motor_1_pwm")
        motor_1_dir = self.devices.get("motor_1_dir")
        motor_2_pwm = self.devices.get("motor_2_pwm")
        motor_2_dir = self.devices.get("motor_2_dir")
        if motor_1_pwm is None or motor_1_dir is None or motor_2_pwm is None or motor_2_dir is None:
            return

        state_a = self.motor_state.setdefault(motor_1_pwm, {"speed": 0.0, "direction": True})
        state_b = self.motor_state.setdefault(motor_2_pwm, {"speed": 0.0, "direction": True})
        current_a = state_a["speed"]
        current_b = state_b["speed"]

        # If either direction changes, ramp both to zero first to reduce stress and keep sync.
        if (current_a > 0 and forward_a != state_a["direction"]) or (
            current_b > 0 and forward_b != state_b["direction"]
        ):
            current_a, current_b = self._ramp_pwm_dual(
                motor_1_pwm, current_a, 0,
                motor_2_pwm, current_b, 0,
                ramp_time / 2, step_interval
            )

        motor_1_dir.value = 1 if forward_a else 0
        motor_2_dir.value = 1 if forward_b else 0

        target_a = max(0.0, min(100.0, speed_a))
        target_b = max(0.0, min(100.0, speed_b))
        current_a, current_b = self._ramp_pwm_dual(
            motor_1_pwm, current_a, target_a,
            motor_2_pwm, current_b, target_b,
            ramp_time, step_interval
        )

        state_a["speed"] = current_a
        state_a["direction"] = forward_a
        state_b["speed"] = current_b
        state_b["direction"] = forward_b

    def stop_motors(self):
        motor_1_pwm = self.devices.get("motor_1_pwm")
        motor_2_pwm = self.devices.get("motor_2_pwm")
        if motor_1_pwm:
            motor_1_pwm.value = 0
        if motor_2_pwm:
            motor_2_pwm.value = 0
        for state in self.motor_state.values():
            state["speed"] = 0.0

    # --- LED helpers ---
    def toggle_led(self, led, timer, interval):
        if led is None:
            return
        if (timer // interval) % 2 == 0:
            led.on()
        else:
            led.off()

    def signal_led(self, times):
        led_red = self.devices.get("led_red")
        led_green = self.devices.get("led_green")
        for _ in range(times):
            if led_red:
                led_red.on()
            if led_green:
                led_green.on()
            time.sleep(0.2)
            if led_red:
                led_red.off()
            if led_green:
                led_green.off()
            time.sleep(0.2)

# --- Navigation calculation ---
def calc_distance_and_azimuth(lat1, lng1, lat2, lng2):
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

# --- Main function ---
def main():
    controller = CanSatController(TARGET_LAT, TARGET_LNG)
    controller.run()

# --- Entry point ---
if __name__ == "__main__":
    main()
