import argparse
import base64
import json
import math
import os
import socket
import struct
import sys
import threading
import time
from pathlib import Path

import cv2
from gpiozero import DigitalOutputDevice, LED, PWMOutputDevice
from gpiozero.pins.lgpio import LGPIOFactory

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from library import bno055
from library import detect_corn as dc

from cansat_mission.constants import (
    BNO_SETUP_RETRY_COUNT,
    BNO_SETUP_RETRY_INTERVAL,
    CAMERA_ACTIVE_SLEEP,
    CAMERA_IDLE_SLEEP,
    CONE_CENTER_POSITION,
    DEVICE_BMP,
    DEVICE_BNO,
    DEVICE_DETECTOR,
    DEVICE_LED_GREEN,
    DEVICE_LED_RED,
    DEVICE_MOTOR_1_DIR,
    DEVICE_MOTOR_1_PWM,
    DEVICE_MOTOR_2_DIR,
    DEVICE_MOTOR_2_PWM,
    DEVICE_SONAR,
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
    PIN_EN1,
    PIN_EN2,
    PIN_LED_GREEN,
    PIN_LED_RED,
    PIN_PH1,
    PIN_PH2,
    PWM_FREQ,
    ROI_PATH_1,
    ROI_PATH_2,
    Phase,
)
from cansat_mission.managers import LedManager, MotorManager, SensorManager
from cansat_mission.phases import Phase4Handler, Phase5Handler, Phase6Handler
from cansat_mission.state import CanSatState


class RelayController(MotorManager, SensorManager, LedManager):
    def __init__(self, args):
        self.args = args
        self.state = CanSatState()
        self.state.update_navigation(phase=int(args.start_phase))

        self.devices = {
            DEVICE_BNO: None,
            DEVICE_BMP: None,
            DEVICE_DETECTOR: None,
            DEVICE_LED_RED: None,
            DEVICE_LED_GREEN: None,
            DEVICE_MOTOR_1_PWM: None,
            DEVICE_MOTOR_1_DIR: None,
            DEVICE_MOTOR_2_PWM: None,
            DEVICE_MOTOR_2_DIR: None,
            DEVICE_SONAR: None,
        }

        self.motor_state = {}
        self.stop_event = threading.Event()
        self.debug_lock = threading.Lock()

        self.frame_b64 = None
        self.frame_seq = 0
        self.camera_debug = {
            "detected": False,
            "centroid_px": None,
            "bbox_px": None,
            "goal_sign": False,
            "message": "init",
        }

        self.roi_img = None
        self.led_blink_timer = 0
        self.searching_flag = False
        self.count_cone_lost = 0
        self.time_phase3_start = 0.0
        self.time_phase5_start = 0.0
        self.time_start_searching_cone = 0.0
        self.time_camera_start = 0.0
        self.camera_fail_count = 0
        self.camera_last_reinit = 0.0
        self.camera_dead_since = None
        self.camera_phase4_attempts = 0
        self.camera_phase5_attempts = 0
        self.camera_phase4_start = None
        self.camera_phase5_start = None

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

        self.phase2_stage = "straight"
        self.phase2_stage_start = None
        self.phase3_no_heading_start = None
        self.obstacle_detect_count = 0

        self.phase_handlers = {
            Phase.PHASE4: Phase4Handler(),
            Phase.PHASE5: Phase5Handler(),
            Phase.PHASE6: Phase6Handler(),
        }
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
            sx += weight * math.cos(rad)
            sy += weight * math.sin(rad)
            total_w += weight

        if total_w <= 0.0:
            return None, HEADING_SOURCE_INVALID, 0.0

        fused = math.degrees(math.atan2(sy, sx))
        if fused < 0.0:
            fused += 360.0
        return fused, HEADING_SOURCE_JOINER.join(source_parts), total_w

    def setup_hardware_minimal(self):
        try:
            bno_dev = bno055.BNO055()
            for i in range(BNO_SETUP_RETRY_COUNT):
                if bno_dev.setUp():
                    self.devices[DEVICE_BNO] = bno_dev
                    print("BNO055: OK")
                    break
                print(f"BNO055: Retry {i + 1}...")
                time.sleep(BNO_SETUP_RETRY_INTERVAL)
        except Exception as exc:
            print(f"BNO055 init error: {exc}")

        try:
            detector = dc.detector()
            roi_img = None
            if os.path.exists(ROI_PATH_1):
                roi_img = cv2.imread(ROI_PATH_1)
            elif os.path.exists(ROI_PATH_2):
                roi_img = cv2.imread(ROI_PATH_2)
            self.roi_img = roi_img
            detector.set_roi_img(roi_img)
            detector.detect_cone()
            self.devices[DEVICE_DETECTOR] = detector
            print("Camera detector: OK")
        except Exception as exc:
            print(f"Camera detector init error: {exc}")
            self.devices[DEVICE_DETECTOR] = None

        try:
            pin_factory = LGPIOFactory()
            self.devices[DEVICE_LED_RED] = LED(PIN_LED_RED, pin_factory=pin_factory)
            self.devices[DEVICE_LED_GREEN] = LED(PIN_LED_GREEN, pin_factory=pin_factory)
            self.devices[DEVICE_MOTOR_1_PWM] = PWMOutputDevice(PIN_EN1, pin_factory=pin_factory, frequency=PWM_FREQ, initial_value=0)
            self.devices[DEVICE_MOTOR_1_DIR] = DigitalOutputDevice(PIN_PH1, pin_factory=pin_factory, initial_value=False)
            self.devices[DEVICE_MOTOR_2_PWM] = PWMOutputDevice(PIN_EN2, pin_factory=pin_factory, frequency=PWM_FREQ, initial_value=0)
            self.devices[DEVICE_MOTOR_2_DIR] = DigitalOutputDevice(PIN_PH2, pin_factory=pin_factory, initial_value=False)
            for key in (DEVICE_MOTOR_1_PWM, DEVICE_MOTOR_2_PWM):
                pwm_dev = self.devices.get(key)
                if pwm_dev:
                    self.motor_state[pwm_dev] = {"speed": 0.0, "direction": True}
            self.stop_motors()
            print("GPIO: OK")
        except Exception as exc:
            print(f"GPIO init error: {exc}")

    def _extract_detector_box(self, detector):
        if detector is None or getattr(detector, "binarized_img", None) is None:
            return None, None
        try:
            nlabels, _, stats, centroids = cv2.connectedComponentsWithStats(detector.binarized_img.astype("uint8"))
            if nlabels <= 1:
                return None, None
            stats = stats[1:]
            centroids = centroids[1:]
            occupancies = stats[:, cv2.CC_STAT_AREA] / float(detector.camera_width * detector.camera_height)
            valid = [i for i, occ in enumerate(occupancies) if occ > 0.001]
            if not valid:
                return None, None
            best_idx = max(valid, key=lambda i: occupancies[i])
            s = stats[best_idx]
            c = centroids[best_idx]
            bbox = [int(s[cv2.CC_STAT_LEFT]), int(s[cv2.CC_STAT_TOP]), int(s[cv2.CC_STAT_WIDTH]), int(s[cv2.CC_STAT_HEIGHT])]
            centroid = [int(c[0]), int(c[1])]
            return bbox, centroid
        except Exception:
            return None, None
    def _annotate_frame(self, frame, snapshot, bbox, centroid):
        h, w = frame.shape[:2]
        cv2.line(frame, (w // 2, 0), (w // 2, h - 1), (255, 255, 0), 1)
        if bbox is not None:
            x, y, bw, bh = bbox
            cv2.rectangle(frame, (x, y), (x + bw, y + bh), (0, 255, 0), 2)
        if centroid is not None:
            cv2.circle(frame, (centroid[0], centroid[1]), 5, (0, 255, 255), -1)
            cv2.putText(frame, f"centroid=({centroid[0]},{centroid[1]})", (10, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 2)
        cv2.putText(
            frame,
            f"phase={snapshot['phase']} prob={snapshot['cone_probability']:.2f} dir={snapshot['cone_direction']:.2f}",
            (10, h - 28),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            (255, 255, 255),
            2,
        )
        if snapshot.get("cone_is_reached", False) or snapshot["phase"] == int(Phase.PHASE6):
            cv2.rectangle(frame, (0, 0), (w - 1, 40), (0, 140, 0), -1)
            cv2.putText(frame, "GOAL REACHED", (10, 28), cv2.FONT_HERSHEY_DUPLEX, 0.9, (255, 255, 255), 2)
        return frame

    def phase_loop(self):
        while not self.stop_event.is_set():
            snapshot = self.state.snapshot()
            phase = Phase(snapshot["phase"])
            handler = self.phase_handlers.get(phase)
            if handler is None:
                time.sleep(0.1)
                continue
            try:
                handler.execute(self, snapshot)
            except SystemExit:
                if self.args.exit_on_goal:
                    self.stop_event.set()
                time.sleep(0.1)

    def camera_debug_loop(self):
        while not self.stop_event.is_set():
            phase = Phase(self.state.snapshot()["phase"])
            if phase in (Phase.PHASE4, Phase.PHASE5, Phase.PHASE6):
                self.cone_detect()
                snapshot = self.state.snapshot()
                detector = self.devices.get(DEVICE_DETECTOR)
                bbox, centroid = self._extract_detector_box(detector)
                msg = "ok" if detector is not None else "detector_unavailable"
                frame_b64 = None
                if detector is not None and getattr(detector, "input_img", None) is not None:
                    frame = detector.input_img.copy()
                    frame = self._annotate_frame(frame, snapshot, bbox, centroid)
                    frame = cv2.resize(frame, (320, 240), interpolation=cv2.INTER_LINEAR)
                    ok, enc = cv2.imencode(
                        ".jpg",
                        frame,
                        [int(cv2.IMWRITE_JPEG_QUALITY), int(self.args.jpeg_quality)],
                    )
                    if ok:
                        frame_b64 = base64.b64encode(enc.tobytes()).decode("ascii")
                with self.debug_lock:
                    self.camera_debug = {
                        "detected": bool(getattr(detector, "is_detected", False)) if detector else False,
                        "centroid_px": centroid,
                        "bbox_px": bbox,
                        "goal_sign": bool(snapshot.get("cone_is_reached", False) or phase == Phase.PHASE6),
                        "message": msg,
                    }
                    if frame_b64 is not None:
                        self.frame_b64 = frame_b64
                        self.frame_seq += 1
                time.sleep(CAMERA_ACTIVE_SLEEP)
            else:
                time.sleep(CAMERA_IDLE_SLEEP)

    def bno_loop(self):
        while not self.stop_event.is_set():
            bno_data = self.get_bno_data()
            if bno_data:
                self.state.update_imu(
                    acc=bno_data["acc"],
                    gyro=bno_data["gyro"],
                    mag=bno_data["mag"],
                    fall=bno_data["fall"],
                    angle=bno_data["angle"],
                    angle_valid=bno_data["angle_valid"],
                )
            else:
                self.state.update_imu(angle_valid=False)
            time.sleep(0.06)

    def _send_packet(self, sock, payload):
        body = json.dumps(payload, ensure_ascii=True, separators=(",", ":")).encode("utf-8")
        sock.sendall(struct.pack(">I", len(body)))
        sock.sendall(body)

    def tx_loop(self):
        sent = 0
        tx_interval = 1.0 / max(1.0, self.args.tx_hz)
        while not self.stop_event.is_set():
            sock = None
            try:
                sock = socket.create_connection((self.args.pc_host, self.args.pc_port), timeout=5.0)
                sock.settimeout(5.0)
                print(f"Connected to PC monitor: {self.args.pc_host}:{self.args.pc_port}")
                while not self.stop_event.is_set():
                    snapshot = self.state.snapshot()
                    with self.debug_lock:
                        debug = dict(self.camera_debug)
                        frame_seq = self.frame_seq
                        frame_b64 = self.frame_b64
                    payload = {
                        "type": "telemetry",
                        "timestamp": time.time(),
                        "phase": snapshot["phase"],
                        "camera": {
                            "cone_probability": snapshot["cone_probability"],
                            "cone_direction": snapshot["cone_direction"],
                            "cone_is_reached": snapshot["cone_is_reached"],
                            "debug": debug,
                        },
                        "bno": {
                            "acc": snapshot["acc"],
                            "gyro": snapshot["gyro"],
                            "mag": snapshot["mag"],
                            "angle": snapshot["angle"],
                            "angle_valid": snapshot["angle_valid"],
                            "fall": snapshot["fall"],
                            "stale_sec": self.bno_stale_sec,
                        },
                        "gps": {
                            "lat": snapshot["lat"],
                            "lng": snapshot["lng"],
                            "gps_detect": snapshot["gps_detect"],
                            "gps_heading": snapshot["gps_heading"],
                            "gps_heading_valid": snapshot["gps_heading_valid"],
                            "num_sats": None,
                            "hdop": None,
                            "gps_qual": None,
                        },
                        "frame_seq": frame_seq,
                    }
                    if sent % max(1, self.args.video_every) == 0:
                        payload["frame_jpeg_b64"] = frame_b64
                    self._send_packet(sock, payload)
                    sent += 1
                    time.sleep(tx_interval)
            except Exception as exc:
                print(f"TX reconnect: {exc}")
                time.sleep(1.0)
            finally:
                if sock is not None:
                    try:
                        sock.close()
                    except Exception:
                        pass
    def run(self):
        self.setup_hardware_minimal()
        threads = [
            threading.Thread(target=self.phase_loop, daemon=True),
            threading.Thread(target=self.move_motor_thread, daemon=True),
            threading.Thread(target=self.gps_thread, daemon=True),
            threading.Thread(target=self.bno_loop, daemon=True),
            threading.Thread(target=self.camera_debug_loop, daemon=True),
            threading.Thread(target=self.tx_loop, daemon=True),
        ]
        for th in threads:
            th.start()

        try:
            while not self.stop_event.is_set():
                time.sleep(1.0)
        except KeyboardInterrupt:
            self.stop_event.set()
        finally:
            self.stop_motors()


def parse_args():
    parser = argparse.ArgumentParser(description="SBC relay (phase4-6 via mission handlers + custom realtime debug relay)")
    parser.add_argument("--pc-host", required=True)
    parser.add_argument("--pc-port", type=int, default=5001)
    parser.add_argument("--jpeg-quality", type=int, default=55)
    parser.add_argument("--tx-hz", type=float, default=10.0)
    parser.add_argument("--video-every", type=int, default=2)
    parser.add_argument("--start-phase", type=int, default=4, choices=[4, 5, 6])
    parser.add_argument("--exit-on-goal", action="store_true")
    return parser.parse_args()


def main():
    args = parse_args()
    args.jpeg_quality = max(1, min(95, args.jpeg_quality))
    args.tx_hz = max(1.0, args.tx_hz)
    args.video_every = max(1, args.video_every)
    RelayController(args).run()


if __name__ == "__main__":
    main()
