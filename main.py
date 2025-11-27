import csv
import datetime
import math
import os
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
from library.micropyGPS import MicropyGPS

# --- Constants ---
TIMEOUT_PHASE_0 = 5 * 60   # 落下判定の最大
TIMEOUT_PHASE_1 = 30       # パラシュート切離の最大
TIMEOUT_PHASE_2 = 2 * 60   # キャリブレーションの最大
TIMEOUT_PHASE_3 = 5 * 60   # GPS誘導の最大
TIMEOUT_PHASE_4 = 60       # コーン探索の最大
TIMEOUT_PHASE_5 = 45       # 接近・スタック判定の最大
DATA_SAMPLING_RATE = 0.06  # センサーデータ取得間隔（秒）

PIN_ENA = 2
PIN_PHA = 13
PIN_ENB = 17
PIN_PHB = 19
PIN_LED_1 = 5
PIN_LED_2 = 6
PIN_TRIG = 23
PIN_ECHO = 24
PWM_FREQ = 20000

TARGET_LAT = 38.26052
TARGET_LNG = 140.8544151


class CanSatState:
    def __init__(self):
        self.lock = threading.Lock()
        self.acc = [0.0, 0.0, 0.0]
        self.gyro = [0.0, 0.0, 0.0]
        self.mag = [0.0, 0.0, 0.0]
        self.lat = 0.0
        self.lng = 0.0
        self.alt = 0.0
        self.pres = 0.0
        self.distance = 0.0
        self.azimuth = 0.0
        self.angle = 0.0
        self.direction = 0.0
        self.fall = 0.0
        self.cone_direction = 0.5
        self.cone_probability = 0.0
        self.obstacle_dist = 999.0
        self.phase = 0
        self.gps_detect = 0
        self.cone_is_reached = False

    def update_imu(self, acc=None, gyro=None, mag=None, fall=None, angle=None):
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

    def update_gps(self, lat=None, lng=None, gps_detect=None):
        with self.lock:
            if lat is not None:
                self.lat = lat
            if lng is not None:
                self.lng = lng
            if gps_detect is not None:
                self.gps_detect = gps_detect

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

    def snapshot(self):
        with self.lock:
            return {
                "acc": list(self.acc),
                "gyro": list(self.gyro),
                "mag": list(self.mag),
                "lat": self.lat,
                "lng": self.lng,
                "alt": self.alt,
                "pres": self.pres,
                "distance": self.distance,
                "azimuth": self.azimuth,
                "angle": self.angle,
                "direction": self.direction,
                "fall": self.fall,
                "cone_direction": self.cone_direction,
                "cone_probability": self.cone_probability,
                "obstacle_dist": self.obstacle_dist,
                "phase": self.phase,
                "gps_detect": self.gps_detect,
                "cone_is_reached": self.cone_is_reached,
            }


def current_milli_time():
    return round(time.time() * 1000)


class CanSatController:
    def __init__(self, target_lat, target_lng):
        self.state = CanSatState()
        self.target_lat = target_lat
        self.target_lng = target_lng
        now_time = datetime.datetime.now()
        os.makedirs("./log", exist_ok=True)
        self.log_path = "./log/robust_log_" + now_time.strftime("%Y-%m%d-%H%M%S") + ".csv"

        self.devices = {}
        self.led_blink_timer = 0
        self.searching_flag = False
        self.count_cone_lost = 0
        self.time_phase3_start = 0
        self.time_phase4_start = 0
        self.time_start_searching_cone = 0
        self.time_camera_start = 0

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
        led1 = self.devices.get("led1")
        led2 = self.devices.get("led2")
        print("phase0 : falling")
        self.toggle_led(led1, self.led_blink_timer, interval=5)
        if led2:
            led2.off()

        start = time.time()
        initial_alt = st["alt"]
        print(f"Start Altitude: {initial_alt:.2f}m")

        while True:
            self.led_blink_timer += 1
            self.toggle_led(led1, self.led_blink_timer, interval=5)

            st_now = self.state.snapshot()
            is_impact = st_now["fall"] > 30.0
            altitude_diff = initial_alt - st_now["alt"]
            is_drop = altitude_diff > 60.0

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
        led1 = self.devices.get("led1")
        led2 = self.devices.get("led2")
        print("phase1 : removing parachute")
        if led1:
            led1.on()
        if led2:
            led2.off()
        self.state.update_navigation(direction=-400.0, phase=1)
        time.sleep(2.0)
        self.state.update_navigation(phase=2)
        self.time_phase2_start = time.time()

    def handle_phase2(self):
        led1 = self.devices.get("led1")
        led2 = self.devices.get("led2")
        bno = self.devices.get("bno")
        print("phase2 : BNO Calibration (Spinning)")
        if led1:
            led1.off()
        if led2:
            led2.on()

        calib_start_time = time.time()
        while True:
            self.led_blink_timer += 1
            self.toggle_led(led1, self.led_blink_timer, interval=3)

            if time.time() - calib_start_time > TIMEOUT_PHASE_2:
                print("Phase2 TIMEOUT: Force Phase 3 (Calibration Incomplete)")
                break

            if bno is not None:
                sys_st, gyro_st, accel_st, mag_st = bno.getCalibrationStatus()
                if self.led_blink_timer % 10 == 0:
                    print(f"Calib Status: Sys={sys_st} Gyro={gyro_st} Acc={accel_st} Mag={mag_st}")
                if mag_st >= 2:
                    print("Calibration OK! (Mag >= 2)")
                    break
            else:
                print("BNO None: Skip Calibration")
                break

            time.sleep(0.1)

        self.state.update_navigation(phase=3)
        self.time_phase3_start = time.time()

    def handle_phase3(self, st):
        led1 = self.devices.get("led1")
        led2 = self.devices.get("led2")
        if led1:
            led1.off()
        self.toggle_led(led2, self.led_blink_timer, interval=10)

        if time.time() - self.time_phase3_start > TIMEOUT_PHASE_3:
            print("Phase3 TIMEOUT: Give up GPS, switching to Camera")
            self.state.update_navigation(phase=4)
            return

        if st["gps_detect"] == 1:
            dist, azi = calc_distance_and_azimuth(st["lat"], st["lng"], self.target_lat, self.target_lng)
            self.state.update_navigation(distance=dist, azimuth=azi, direction=azi)
            if self.led_blink_timer % 10 == 0:
                print(f"GPS Nav: Dist={dist:.1f}m, TargetDir={azi:.1f}, MyHead={st['angle']:.1f}")
            if dist < 5.0:
                print(f"Close enough ({dist:.1f}m): Switching to Camera")
                self.state.update_navigation(phase=4)
            self.toggle_led(led2, self.led_blink_timer, interval=2)
        else:
            if self.led_blink_timer % 20 == 0:
                print("GPS Lost: Keep going...")

    def handle_phase4(self):
        led1 = self.devices.get("led1")
        led2 = self.devices.get("led2")
        st = self.state.snapshot()
        cone_prob = st["cone_probability"]

        print("phase4 : camera searching")
        if led1:
            led1.off()
        if led2:
            led2.on()

        if not self.searching_flag:
            self.searching_flag = True
            self.time_start_searching_cone = time.time()
        else:
            if time.time() - self.time_start_searching_cone >= TIMEOUT_PHASE_4:
                print("Camera TIMEOUT: Cone not found or Camera dead")
                self.searching_flag = False
                self.state.update_navigation(phase=3)
                self.time_phase3_start = time.time()

        if cone_prob > 0.1:
            self.state.update_navigation(phase=5)

    def handle_phase5(self):
        led1 = self.devices.get("led1")
        led2 = self.devices.get("led2")
        print("phase5 : approaching")
        self.time_camera_start = time.time()
        self.count_cone_lost = 0

        while True:
            self.led_blink_timer += 1
            if (self.led_blink_timer // 2) % 2 == 0:
                if led1:
                    led1.on()
                if led2:
                    led2.off()
            else:
                if led1:
                    led1.off()
                if led2:
                    led2.on()

            st = self.state.snapshot()

            is_det = st["cone_probability"] > 0.1
            is_reach = st["cone_is_reached"]

            if not is_det:
                self.count_cone_lost += 1
            else:
                self.count_cone_lost = 0

            if self.count_cone_lost >= 10:
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
        led1 = self.devices.get("led1")
        led2 = self.devices.get("led2")
        print("phase6 : Goal!!")
        if led1:
            led1.on()
        if led2:
            led2.on()
        self.stop_motors()
        sys.exit()

    # --- Hardware and threads ---
    def setup_hardware(self):
        print("--- Robust Setup Start ---")
        self.devices = {
            "bno": None,
            "bmp": None,
            "detector": None,
            "led1": None,
            "led2": None,
            "motor_a_pwm": None,
            "motor_a_dir": None,
            "motor_b_pwm": None,
            "motor_b_dir": None,
            "sonar": None,
        }

        try:
            bno = bno055.BNO055()
            for i in range(3):
                if bno.setUp():
                    self.devices["bno"] = bno
                    print("BNO055: OK")
                    break
                print(f"BNO055: Retry {i+1}...")
                time.sleep(0.5)
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
            roi_path_1 = "./log/captured_roi_img.png"
            roi_path_2 = "./log/captured.png"
            roi_img = None
            if os.path.exists(roi_path_1):
                print(f"Loading ROI from {roi_path_1}")
                roi_img = cv2.imread(roi_path_1)
            elif os.path.exists(roi_path_2):
                print(f"Loading ROI from {roi_path_2}")
                roi_img = cv2.imread(roi_path_2)
            else:
                print("WARNING: No ROI image found. Switching to DEFAULT ORANGE detection.")

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
            self.devices["led1"] = LED(PIN_LED_1, pin_factory=pin_factory)
            self.devices["led2"] = LED(PIN_LED_2, pin_factory=pin_factory)
            self.devices["motor_a_pwm"] = PWMOutputDevice(PIN_ENA, pin_factory=pin_factory, frequency=PWM_FREQ, initial_value=0)
            self.devices["motor_a_dir"] = DigitalOutputDevice(PIN_PHA, pin_factory=pin_factory, initial_value=False)
            self.devices["motor_b_pwm"] = PWMOutputDevice(PIN_ENB, pin_factory=pin_factory, frequency=PWM_FREQ, initial_value=0)
            self.devices["motor_b_dir"] = DigitalOutputDevice(PIN_PHB, pin_factory=pin_factory, initial_value=False)
            self.devices["sonar"] = DistanceSensor(echo=PIN_ECHO, trigger=PIN_TRIG, max_distance=4.0, pin_factory=pin_factory)
            self.stop_motors()
            print("GPIOZero: OK")
        except Exception as e:
            print(f"GPIOZero Setup Error {e}.")

        self.start_threads()
        self.init_log_file()
        print("--- Setup Finished (Ready to Die Trying) ---")

    def start_threads(self):
        try:
            threading.Thread(target=self.move_motor_thread, daemon=True).start()
            threading.Thread(target=self.data_thread, daemon=True).start()
            threading.Thread(target=self.gps_thread, daemon=True).start()
            threading.Thread(target=self.camera_thread, daemon=True).start()
        except Exception as e:
            print(f"Thread Start Error {e}.")

    def init_log_file(self):
        try:
            with open(self.log_path, "w", newline="") as f:
                writer = csv.writer(f)
                writer.writerow([
                    "MilliTime", "Phase",
                    "AccX", "AccY", "AccZ", "GyroX", "GyroY", "GyroZ", "MagX", "MagY", "MagZ",
                    "LAT", "LNG", "ALT", "Pres",
                    "Distance", "Azimuth", "Angle", "Direction", "Fall",
                    "ConeDir", "ConeProb", "ObstacleDist"
                ])
        except Exception:
            print("Log File Init Failed. No logging.")

    # --- Sensor readers ---
    def get_bno_data(self):
        bno_instance = self.devices.get("bno")
        if bno_instance is None:
            return None
        try:
            acc = bno_instance.getAcc() or [0, 0, 0]
            gyro = bno_instance.getGyro() or [0, 0, 0]
            mag = bno_instance.getMag() or [0, 0, 0]
            fall = math.sqrt(acc[0] ** 2 + acc[1] ** 2 + acc[2] ** 2)
            euler = bno_instance.getEuler()
            angle = euler[0] if euler else 0.0
            return {"acc": acc, "gyro": gyro, "mag": mag, "fall": fall, "angle": angle}
        except Exception:
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
            if dist_m is not None and 0 < dist_m < 4.0:
                return dist_m * 100.0
        except Exception:
            pass
        return None

    def cone_detect(self):
        detector = self.devices.get("detector")
        if detector is None:
            self.state.update_cone(cone_direction=0.5, cone_probability=0.0, cone_is_reached=False)
            return
        try:
            detector.detect_cone()
            prob = detector.probability if detector.probability else 0.0
            cdir = 0.5
            if detector.cone_direction is not None:
                cdir = 1.0 - detector.cone_direction
            self.state.update_cone(cone_direction=cdir, cone_probability=prob, cone_is_reached=detector.is_reached)
        except Exception:
            self.state.update_cone(cone_direction=0.5, cone_probability=0.0, cone_is_reached=False)

    # --- Thread loops ---
    def gps_thread(self):
        s = None
        try:
            s = serial.Serial("/dev/serial0", 115200, timeout=1)
        except Exception:
            print("GPS Serial Open Failed. GPS is DEAD.")

        gps = MicropyGPS(9, "dd")

        while True:
            if s is None:
                time.sleep(1)
                continue
            try:
                line = s.readline().decode("utf-8", errors="ignore")
                if len(line) > 0 and line[0] == "$":
                    for x in line:
                        gps.update(x)
                    lat = gps.latitude[0]
                    lng = gps.longitude[0]
                    is_detect = 1 if lat != 0.0 else 0
                    self.state.update_gps(lat=lat, lng=lng, gps_detect=is_detect)
            except Exception:
                pass

    def camera_thread(self):
        while True:
            current_phase = self.state.snapshot()["phase"]
            if current_phase in [4, 5]:
                self.cone_detect()
                time.sleep(0.05)
            else:
                time.sleep(0.5)

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
                )

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
                    ])
            except Exception as e:
                print(f"Log Error: {e}")

            time.sleep(DATA_SAMPLING_RATE)

    def move_motor_thread(self):
        BASE_SPEED = 60
        AVOID_DIST = 30.0

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

            if phase not in [0, 1, 5, 6] and obstacle_dist < AVOID_DIST:
                print(f"Obstacle Detected! {obstacle_dist:.1f}cm")
                self.stop_motors()
                time.sleep(0.2)
                self.set_motor(self.devices["motor_a_pwm"], self.devices["motor_a_dir"], 60, False)
                self.set_motor(self.devices["motor_b_pwm"], self.devices["motor_b_dir"], 60, False)
                time.sleep(1.0)
                self.set_motor(self.devices["motor_a_pwm"], self.devices["motor_a_dir"], 60, False)
                self.set_motor(self.devices["motor_b_pwm"], self.devices["motor_b_dir"], 60, True)
                time.sleep(0.5)
                self.stop_motors()
                time.sleep(0.2)
                continue

            if phase == 1 and direction == -400.0:
                separation_speed = 100
                self.set_motor(self.devices["motor_a_pwm"], self.devices["motor_a_dir"], separation_speed, True)
                self.set_motor(self.devices["motor_b_pwm"], self.devices["motor_b_dir"], separation_speed, True)
                time.sleep(0.05)
                continue

            if phase == 2:
                calib_speed = 50
                self.set_motor(self.devices["motor_a_pwm"], self.devices["motor_a_dir"], calib_speed, True)
                self.set_motor(self.devices["motor_b_pwm"], self.devices["motor_b_dir"], calib_speed, False)
                time.sleep(0.05)
                continue

            if phase == 3:
                target_heading = direction - 5
                current_heading = angle
                diff = target_heading - current_heading
                if diff > 180:
                    diff -= 360
                if diff < -180:
                    diff += 360
                turn_val = diff * 0.5
                turn_val = max(-30, min(30, turn_val))
                speed_l = max(0, min(100, BASE_SPEED + turn_val))
                speed_r = max(0, min(100, BASE_SPEED - turn_val))
                self.set_motor(self.devices["motor_a_pwm"], self.devices["motor_a_dir"], speed_r, True)
                self.set_motor(self.devices["motor_b_pwm"], self.devices["motor_b_dir"], speed_l, True)

            elif phase == 4:
                search_speed = 40
                self.set_motor(self.devices["motor_a_pwm"], self.devices["motor_a_dir"], search_speed, True)
                self.set_motor(self.devices["motor_b_pwm"], self.devices["motor_b_dir"], search_speed, False)

            elif phase == 5:
                center = 0.5
                err = cone_direction - center
                turn_cam = err * 80
                speed_l = max(0, min(100, BASE_SPEED + turn_cam))
                speed_r = max(0, min(100, BASE_SPEED - turn_cam))
                self.set_motor(self.devices["motor_a_pwm"], self.devices["motor_a_dir"], speed_r, True)
                self.set_motor(self.devices["motor_b_pwm"], self.devices["motor_b_dir"], speed_l, True)

            time.sleep(0.05)

    # --- Actuators ---
    def set_motor(self, motor_pwm, motor_dir, speed, forward):
        if motor_pwm is None or motor_dir is None:
            return
        motor_dir.value = 1 if forward else 0
        motor_pwm.value = max(0.0, min(1.0, speed / 100.0))

    def stop_motors(self):
        motor_a_pwm = self.devices.get("motor_a_pwm")
        motor_b_pwm = self.devices.get("motor_b_pwm")
        if motor_a_pwm:
            motor_a_pwm.value = 0
        if motor_b_pwm:
            motor_b_pwm.value = 0

    # --- LED helpers ---
    def toggle_led(self, led, timer, interval):
        if led is None:
            return
        if (timer // interval) % 2 == 0:
            led.on()
        else:
            led.off()

    def signal_led(self, times):
        led1 = self.devices.get("led1")
        led2 = self.devices.get("led2")
        for _ in range(times):
            if led1:
                led1.on()
            if led2:
                led2.on()
            time.sleep(0.2)
            if led1:
                led1.off()
            if led2:
                led2.off()
            time.sleep(0.2)


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


def main():
    controller = CanSatController(TARGET_LAT, TARGET_LNG)
    controller.run()


if __name__ == "__main__":
    main()
