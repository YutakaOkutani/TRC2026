import argparse
import base64
import json
import math
import signal
import socket
import struct
import sys
import threading
import time
from pathlib import Path

import cv2

PROJECT_ROOT = Path(__file__).resolve().parents[2]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from csmn.const import (
    CAMERA_ACTIVE_SLEEP,
    CAMERA_IDLE_SLEEP,
    CONE_PROBABILITY_THRESHOLD_PHASE4,
    CONE_PROBABILITY_THRESHOLD_PHASE5,
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
    TARGET_LAT,
    TARGET_LNG,
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
    Phase,
)
from csmn.mgr import HardwareManager, LedManager, MotorManager, SensorManager
from csmn.phs import Phase4Handler, Phase5Handler, Phase6Handler, Phase7Handler
from csmn.st import CanSatState

# Runtime defaults (change here when your environment changes)
# Variable parameter: PC monitor host (receiver IP)
DEFAULT_PC_HOST = "100.100.219.60"
# Variable parameter: PC monitor port
DEFAULT_PC_PORT = 5001
# Variable parameter: telemetry transmit rate [Hz]
DEFAULT_TX_HZ = 10.0
# Variable parameter: attach one JPEG frame every N packets
DEFAULT_VIDEO_EVERY = 2
# Variable parameter: JPEG quality (1-95)
DEFAULT_JPEG_QUALITY = 55
# Variable parameter: start phase for this relay test
DEFAULT_START_PHASE = 4


class RelayController(HardwareManager, MotorManager, SensorManager, LedManager):
    def __init__(self, args):
        self.args = args
        self.target_lat = float(getattr(args, "target_lat", TARGET_LAT))
        self.target_lng = float(getattr(args, "target_lng", TARGET_LNG))
        self.st = CanSatState()
        self.st.update_navigation(phase=int(args.start_phase))
        self.phase_entry_time = time.time()
        self.last_phase_observed = Phase(int(args.start_phase))
        self.phase5_entry_marker = None
        self.mission_end_reason = "RUNNING"
        self.shutdown_reason = None
        self._shutdown_packet_sent = False
        self._shutdown_requested = False
        self.phase6_exit_started_at = None
        self._last_logged_cone_method = None

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
        # Detector input is standardized to the vehicle-forward orientation in detect_corn.py.
        # Keep optional override, but default to no extra control inversion.
        self.camera_control_invert_x = bool(getattr(args, "camera_control_invert_x", False))
        print(f"Camera control X inversion: {'ON' if self.camera_control_invert_x else 'OFF'}")

        self.frame_b64 = None
        self.frame_seq = 0
        self.camera_debug = {
            "detected": False,
            "centroid_px": None,
            "bbox_px": None,
            "goal_sign": False,
            "message": "init",
            "scores": {},
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
            Phase.PHASE7: Phase7Handler(),
        }

    def request_shutdown(self, reason="SBC_STOP"):
        if self._shutdown_requested:
            return
        self._shutdown_requested = True
        self.shutdown_reason = self.shutdown_reason or str(reason)
        self.stop_event.set()
        try:
            self.stop_motors()
        except Exception as exc:
            print(f"Emergency stop failed: {exc}")

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
        self._setup_bno_device()
        self._setup_camera_detector()
        self._setup_gpio_devices(include_sonar=False)

    def _extract_detector_box(self, detector):
        if detector is None:
            return None, None
        bbox = getattr(detector, "detected", None)
        centroid = getattr(detector, "centroids", None)
        if bbox is not None:
            bbox = [int(v) for v in bbox]
        if centroid is not None:
            try:
                centroid = [int(centroid[0]), int(centroid[1])]
            except Exception:
                centroid = None
        if bbox is not None or centroid is not None:
            return bbox, centroid
        if getattr(detector, "binarized_img", None) is None:
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

    def _prepare_preview_frame(self, frame):
        if frame is None:
            return None
        # Optional preview transforms for operator convenience.
        # Detector input is already standardized in detect_corn.py.
        if getattr(self.args, "preview_swap_rb", True):
            frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        if getattr(self.args, "preview_rotate_180", True):
            frame = cv2.rotate(frame, cv2.ROTATE_180)
        return frame

    def _annotate_frame(self, frame, snapshot, bbox, centroid):
        if bbox is not None:
            x, y, bw, bh = bbox
            cv2.rectangle(frame, (x, y), (x + bw, y + bh), (0, 255, 255), 2)
        return frame

    def _overlay_debug_text(self, frame, snapshot, detector, phase):
        if frame is None:
            return None
        prob = float(snapshot.get("cone_probability", 0.0))
        scores = dict(getattr(detector, "debug_scores", {}) or {}) if detector is not None else {}
        shape = float(scores.get("shape", 0.0))
        sv = float(scores.get("sv", 0.0))
        hue = float(scores.get("hue", 0.0))
        roi_support = float(scores.get("roi_support", 0.0))
        swap_used = int(scores.get("swap_used", 0.0))
        as_is_prob = float(scores.get("as_is_prob", 0.0))
        swap_prob = float(scores.get("swap_prob", 0.0))
        as_is_shape = float(scores.get("as_is_shape", 0.0))
        swap_shape = float(scores.get("swap_shape", 0.0))
        as_is_select = float(scores.get("as_is_select", 0.0))
        swap_select = float(scores.get("swap_select", 0.0))
        method = str(getattr(detector, "debug_method", "unknown")) if detector is not None else "detector_unavailable"
        if phase == Phase.PHASE4:
            phase_thr = CONE_PROBABILITY_THRESHOLD_PHASE4
        elif phase == Phase.PHASE5:
            phase_thr = CONE_PROBABILITY_THRESHOLD_PHASE5
        else:
            phase_thr = 0.0
        pass_mark = "PASS" if prob >= phase_thr else "HOLD"
        reject_hints = []
        if prob < phase_thr:
            if shape < 0.45:
                reject_hints.append("shape")
            if sv < 0.35:
                reject_hints.append("sv")
            if hue < 0.45:
                reject_hints.append("hue")
            if not reject_hints:
                reject_hints.append("confirm/center")
        lines = [
            f"prob {prob:.2f}  thr {phase_thr:.2f}  {pass_mark}",
            f"shape {shape:.2f}  sv {sv:.2f}  hue {hue:.2f}  roi {roi_support:.2f}",
            f"as {as_is_prob:.2f}/{as_is_shape:.2f}/{as_is_select:.2f}  sw {swap_prob:.2f}/{swap_shape:.2f}/{swap_select:.2f}",
            f"variant {'swap_rb' if swap_used else 'as_is'}",
            f"det {bool(getattr(detector, 'is_detected', False)) if detector else False}  method {method}",
        ]
        if reject_hints:
            lines.append("weak: " + ",".join(reject_hints))
        y = 18
        for line in lines:
            cv2.putText(frame, line, (8, y), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 0), 3, cv2.LINE_AA)
            cv2.putText(frame, line, (8, y), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (60, 255, 60), 1, cv2.LINE_AA)
            y += 18
        return frame

    def phase_loop(self):
        while not self.stop_event.is_set():
            snapshot = self.st.snapshot()
            phase = Phase(snapshot["phase"])
            if phase != getattr(self, "last_phase_observed", None):
                self.last_phase_observed = phase
                self.phase_entry_time = time.time()
                if phase != Phase.PHASE4:
                    self.searching_flag = False
            handler = self.phase_handlers.get(phase)
            if handler is None:
                time.sleep(0.1)
                continue
            try:
                handler.execute(self, snapshot)
            except SystemExit:
                current_phase = Phase(self.st.snapshot()["phase"])
                if current_phase == Phase.PHASE7:
                    if self.phase6_exit_started_at is None:
                        self.phase6_exit_started_at = time.time()
                        if self.shutdown_reason is None:
                            self.shutdown_reason = self.mission_end_reason if self.mission_end_reason != "RUNNING" else "PHASE7_REACHED"
                        print(f"Final phase reached. Auto exit in {self.args.phase6_hold_sec:.1f}s")
                    elif time.time() - self.phase6_exit_started_at >= self.args.phase6_hold_sec:
                        self.request_shutdown(self.shutdown_reason or "PHASE6_HOLD_COMPLETE")
                elif self.args.exit_on_goal:
                    self.request_shutdown(self.shutdown_reason or "EXIT_ON_GOAL")
                time.sleep(0.1)

    def camera_debug_loop(self):
        while not self.stop_event.is_set():
            phase = Phase(self.st.snapshot()["phase"])
            if phase in (Phase.PHASE4, Phase.PHASE5, Phase.PHASE6, Phase.PHASE7):
                self.cone_detect()
                snapshot = self.st.snapshot()
                detector = self.devices.get(DEVICE_DETECTOR)
                bbox, centroid = self._extract_detector_box(detector)
                msg = "ok" if detector is not None else "detector_unavailable"
                detector_method = str(getattr(detector, "debug_method", "unknown")) if detector is not None else "detector_unavailable"
                if detector_method != self._last_logged_cone_method:
                    print(f"Cone detector method: {detector_method}")
                    self._last_logged_cone_method = detector_method
                frame_b64 = None
                if detector is not None and getattr(detector, "input_img", None) is not None:
                    frame = detector.input_img.copy()
                    frame = self._annotate_frame(frame, snapshot, bbox, centroid)
                    frame = self._prepare_preview_frame(frame)
                    frame = self._overlay_debug_text(frame, snapshot, detector, phase)
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
                        "goal_sign": bool(snapshot.get("cone_is_reached", False) or phase in (Phase.PHASE6, Phase.PHASE7)),
                        "message": msg,
                        "method": detector_method,
                        "scores": dict(getattr(detector, "debug_scores", {}) or {}) if detector else {},
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
                self.st.update_imu(
                    acc=bno_data["acc"],
                    gyro=bno_data["gyro"],
                    mag=bno_data["mag"],
                    fall=bno_data["fall"],
                    angle=bno_data["angle"],
                    angle_valid=bno_data["angle_valid"],
                )
            else:
                self.st.update_imu(angle_valid=False)
            time.sleep(0.06)

    def _send_packet(self, sock, payload):
        body = json.dumps(payload, ensure_ascii=True, separators=(",", ":")).encode("utf-8")
        sock.sendall(struct.pack(">I", len(body)))
        sock.sendall(body)

    def _send_shutdown_packet(self, sock):
        if sock is None or self._shutdown_packet_sent:
            return
        reason = self.shutdown_reason or self.mission_end_reason or "SBC_STOP"
        self._send_packet(
            sock,
            {
                "type": "shutdown",
                "timestamp": time.time(),
                "reason": str(reason),
            },
        )
        self._shutdown_packet_sent = True
        print(f"Sent shutdown packet to PC monitor (reason={reason})")

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
                    snapshot = self.st.snapshot()
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
                if self.stop_event.is_set():
                    try:
                        self._send_shutdown_packet(sock)
                    except Exception as exc:
                        print(f"Shutdown packet send failed: {exc}")
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
            self.request_shutdown("CTRL_C")
        finally:
            self.request_shutdown(self.shutdown_reason or "RUN_EXIT")


def parse_args():
    parser = argparse.ArgumentParser(description="SBC relay (p4-6 via mission handlers + custom realtime debug relay)")
    parser.add_argument("--pc-host", default=DEFAULT_PC_HOST)
    parser.add_argument("--pc-port", type=int, default=DEFAULT_PC_PORT)
    parser.add_argument("--jpeg-quality", type=int, default=DEFAULT_JPEG_QUALITY)
    parser.add_argument("--tx-hz", type=float, default=DEFAULT_TX_HZ)
    parser.add_argument("--video-every", type=int, default=DEFAULT_VIDEO_EVERY)
    parser.add_argument("--start-phase", type=int, default=DEFAULT_START_PHASE, choices=[4, 5, 6, 7])
    parser.add_argument("--exit-on-goal", action="store_true")
    parser.add_argument("--p6-hold-sec", type=float, default=5.0)
    parser.add_argument("--target-lat", type=float, default=TARGET_LAT)
    parser.add_argument("--target-lng", type=float, default=TARGET_LNG)
    parser.add_argument("--preview-rotate-180", action=argparse.BooleanOptionalAction, default=False)
    parser.add_argument("--preview-swap-rb", action=argparse.BooleanOptionalAction, default=False)
    parser.add_argument("--camera-control-invert-x", action=argparse.BooleanOptionalAction, default=False)
    return parser.parse_args()


def main():
    args = parse_args()
    args.jpeg_quality = max(1, min(95, args.jpeg_quality))
    args.tx_hz = max(1.0, args.tx_hz)
    args.video_every = max(1, args.video_every)
    args.phase6_hold_sec = max(0.0, float(args.phase6_hold_sec))
    controller = RelayController(args)

    def _handle_signal(signum, _frame):
        try:
            signame = signal.Signals(signum).name
        except Exception:
            signame = f"SIGNAL_{signum}"
        print(f"\nReceived {signame}. Emergency stop requested.")
        controller.request_shutdown(signame)

    signal.signal(signal.SIGINT, _handle_signal)
    if hasattr(signal, "SIGTERM"):
        signal.signal(signal.SIGTERM, _handle_signal)

    controller.run()


if __name__ == "__main__":
    main()
