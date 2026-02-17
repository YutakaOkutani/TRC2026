import csv
import os
import threading
import time

import cv2
from gpiozero import DigitalOutputDevice, DistanceSensor, LED, PWMOutputDevice
from gpiozero.pins.lgpio import LGPIOFactory

from library import bmp180, bno055
from library import detect_corn as dc

from cansat_mission.constants import (
    BNO_SETUP_RETRY_COUNT,
    BNO_SETUP_RETRY_INTERVAL,
    DEVICE_BMP,
    DEVICE_BNO,
    DEVICE_DETECTOR,
    DEVICE_KEYS,
    DEVICE_LED_GREEN,
    DEVICE_LED_RED,
    DEVICE_MOTOR_1_DIR,
    DEVICE_MOTOR_1_PWM,
    DEVICE_MOTOR_2_DIR,
    DEVICE_MOTOR_2_PWM,
    DEVICE_SONAR,
    LOG_HEADER,
    PIN_ECHO,
    PIN_EN1,
    PIN_EN2,
    PIN_LED_GREEN,
    PIN_LED_RED,
    PIN_PH1,
    PIN_PH2,
    PIN_TRIG,
    PWM_FREQ,
    ROI_PATH_1,
    ROI_PATH_2,
    SONAR_MAX_DISTANCE,
)


class HardwareManager:
    def setup_hardware(self):
        print("--- Robust Setup Start ---")
        self.devices = {key: None for key in DEVICE_KEYS}

        try:
            bno = bno055.BNO055()
            for i in range(BNO_SETUP_RETRY_COUNT):
                if bno.setUp():
                    self.devices[DEVICE_BNO] = bno
                    print("BNO055: OK")
                    break
                print(f"BNO055: Retry {i + 1}...")
                time.sleep(BNO_SETUP_RETRY_INTERVAL)
            else:
                print("WARNING: BNO055 Init Failed.")
        except Exception as exc:
            print(f"BNO055: Critical Error {exc}.")

        try:
            bmp = bmp180.BMP180(oss=3)
            if bmp.setUp():
                self.devices[DEVICE_BMP] = bmp
                print("BMP180: OK")
            else:
                print("WARNING: BMP180 Init Failed.")
        except Exception as exc:
            print(f"BMP180: Critical Error {exc}.")

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
            self.devices[DEVICE_DETECTOR] = detector
            print("Camera: OK (Initialized)")
        except Exception as exc:
            print(f"Camera: Critical Init Error {exc}. Proceeding without Vision.")
            self.devices[DEVICE_DETECTOR] = None

        print("GPIOZero: Initializing devices...")
        try:
            pin_factory = LGPIOFactory()
            self.devices[DEVICE_LED_RED] = LED(PIN_LED_RED, pin_factory=pin_factory)
            self.devices[DEVICE_LED_GREEN] = LED(PIN_LED_GREEN, pin_factory=pin_factory)
            self.devices[DEVICE_MOTOR_1_PWM] = PWMOutputDevice(
                PIN_EN1,
                pin_factory=pin_factory,
                frequency=PWM_FREQ,
                initial_value=0,
            )
            self.devices[DEVICE_MOTOR_1_DIR] = DigitalOutputDevice(
                PIN_PH1,
                pin_factory=pin_factory,
                initial_value=False,
            )
            self.devices[DEVICE_MOTOR_2_PWM] = PWMOutputDevice(
                PIN_EN2,
                pin_factory=pin_factory,
                frequency=PWM_FREQ,
                initial_value=0,
            )
            self.devices[DEVICE_MOTOR_2_DIR] = DigitalOutputDevice(
                PIN_PH2,
                pin_factory=pin_factory,
                initial_value=False,
            )
            self.devices[DEVICE_SONAR] = DistanceSensor(
                echo=PIN_ECHO,
                trigger=PIN_TRIG,
                max_distance=SONAR_MAX_DISTANCE,
                pin_factory=pin_factory,
            )
            self.motor_state = {}
            for key in (DEVICE_MOTOR_1_PWM, DEVICE_MOTOR_2_PWM):
                pwm_dev = self.devices.get(key)
                if pwm_dev:
                    self.motor_state[pwm_dev] = {"speed": 0.0, "direction": True}
            self.stop_motors()
            print("GPIOZero: OK")
        except Exception as exc:
            print(f"GPIOZero Setup Error {exc}.")

        self.start_threads()
        self.init_log_file()
        print("--- Setup Finished (Ready to Die Trying) ---")

    def start_threads(self):
        try:
            threading.Thread(target=self.move_motor_thread, daemon=True).start()
            threading.Thread(target=self.data_thread, daemon=True).start()
            threading.Thread(target=self.gps_thread, daemon=True).start()
            threading.Thread(target=self.camera_thread, daemon=True).start()
        except Exception as exc:
            print(f"Thread Start Error {exc}.")

    def init_log_file(self):
        try:
            with open(self.log_path, "w", newline="") as file_obj:
                writer = csv.writer(file_obj)
                writer.writerow(LOG_HEADER)
        except Exception:
            print("Log File Init Failed. No logging.")
