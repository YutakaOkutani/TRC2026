import argparse
import base64
import json
import socket
import struct
import threading
import time
from collections import deque

import cv2
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import numpy as np


def recv_exact(sock, n):
    data = bytearray()
    while len(data) < n:
        chunk = sock.recv(n - len(data))
        if not chunk:
            return None
        data.extend(chunk)
    return bytes(data)


class MonitorState:
    def __init__(self, history_sec):
        maxlen = int(max(10, history_sec * 20))
        self.lock = threading.Lock()
        self.t = deque(maxlen=maxlen)
        self.bno_acc_norm = deque(maxlen=maxlen)
        self.bno_gyro_norm = deque(maxlen=maxlen)
        self.bno_mag_norm = deque(maxlen=maxlen)
        self.bno_angle = deque(maxlen=maxlen)
        self.gps_sats = deque(maxlen=maxlen)
        self.gps_hdop = deque(maxlen=maxlen)
        self.gps_detect = deque(maxlen=maxlen)
        self.last_frame = None
        self.last_packet_time = 0.0
        self.last_summary = {}

    def update(self, packet):
        ts = packet.get("timestamp", time.time())
        bno = packet.get("bno", {})
        gps = packet.get("gps", {})

        acc = bno.get("acc", [0.0, 0.0, 0.0])
        gyro = bno.get("gyro", [0.0, 0.0, 0.0])
        mag = bno.get("mag", [0.0, 0.0, 0.0])
        angle = bno.get("angle", 0.0)

        acc_norm = float(np.linalg.norm(np.array(acc, dtype=float)))
        gyro_norm = float(np.linalg.norm(np.array(gyro, dtype=float)))
        mag_norm = float(np.linalg.norm(np.array(mag, dtype=float)))

        sats = gps.get("num_sats")
        hdop = gps.get("hdop")
        detect = gps.get("gps_detect", 0)

        frame_b64 = packet.get("frame_jpeg_b64")
        frame = None
        if frame_b64:
            try:
                raw = base64.b64decode(frame_b64.encode("ascii"))
                arr = np.frombuffer(raw, dtype=np.uint8)
                frame = cv2.imdecode(arr, cv2.IMREAD_COLOR)
            except Exception:
                frame = None

        with self.lock:
            self.t.append(ts)
            self.bno_acc_norm.append(acc_norm)
            self.bno_gyro_norm.append(gyro_norm)
            self.bno_mag_norm.append(mag_norm)
            self.bno_angle.append(float(angle))
            self.gps_sats.append(float(sats) if sats is not None else np.nan)
            self.gps_hdop.append(float(hdop) if hdop is not None else np.nan)
            self.gps_detect.append(float(detect))
            if frame is not None:
                self.last_frame = frame
            self.last_packet_time = time.time()
            self.last_summary = {
                "phase": packet.get("phase", -1),
                "cone_probability": packet.get("camera", {}).get("cone_probability", 0.0),
                "cone_direction": packet.get("camera", {}).get("cone_direction", 0.5),
                "cone_is_reached": packet.get("camera", {}).get("cone_is_reached", False),
                "camera_debug": packet.get("camera", {}).get("debug", {}),
                "angle_valid": bno.get("angle_valid", False),
                "gps_detect": gps.get("gps_detect", 0),
                "gps_heading_valid": gps.get("gps_heading_valid", False),
                "gps_lat": gps.get("lat", 0.0),
                "gps_lng": gps.get("lng", 0.0),
            }


class RelayServer:
    def __init__(self, host, port, state):
        self.host = host
        self.port = port
        self.state = state
        self.stop_event = threading.Event()

    def run(self):
        server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server.bind((self.host, self.port))
        server.listen(1)
        print(f"Listening on {self.host}:{self.port}")

        try:
            while not self.stop_event.is_set():
                conn, addr = server.accept()
                print(f"Connected: {addr[0]}:{addr[1]}")
                with conn:
                    conn.settimeout(5.0)
                    while not self.stop_event.is_set():
                        header = recv_exact(conn, 4)
                        if header is None:
                            break
                        size = struct.unpack(">I", header)[0]
                        body = recv_exact(conn, size)
                        if body is None:
                            break
                        try:
                            packet = json.loads(body.decode("utf-8"))
                        except Exception:
                            continue
                        if packet.get("type") == "telemetry":
                            self.state.update(packet)
                print("Disconnected. Waiting for reconnect...")
        finally:
            server.close()


def start_ui(state, history_sec):
    fig, axes = plt.subplots(2, 2, figsize=(10, 6))
    fig.suptitle("CanSat Camera/BNO055/GPS Realtime Monitor")

    line_acc, = axes[0, 0].plot([], [], label="|acc|")
    line_gyro, = axes[0, 0].plot([], [], label="|gyro|")
    axes[0, 0].set_title("BNO Norm")
    axes[0, 0].legend(loc="upper right")

    line_mag, = axes[0, 1].plot([], [], label="|mag|", color="tab:green")
    line_ang, = axes[0, 1].plot([], [], label="angle(deg)", color="tab:orange")
    axes[0, 1].set_title("BNO Mag/Angle")
    axes[0, 1].legend(loc="upper right")

    line_sats, = axes[1, 0].plot([], [], label="satellites", color="tab:blue")
    line_hdop, = axes[1, 0].plot([], [], label="HDOP", color="tab:red")
    axes[1, 0].set_title("GPS Quality")
    axes[1, 0].legend(loc="upper right")

    line_detect, = axes[1, 1].plot([], [], label="gps_detect", color="tab:purple")
    axes[1, 1].set_title("GPS Detect")
    axes[1, 1].set_ylim(-0.1, 1.1)
    axes[1, 1].legend(loc="upper right")

    status_text = fig.text(0.01, 0.01, "waiting...", fontsize=9)

    def update(_):
        with state.lock:
            if len(state.t) == 0:
                return line_acc, line_gyro, line_mag, line_ang, line_sats, line_hdop, line_detect, status_text
            t0 = state.t[0]
            x = [v - t0 for v in state.t]
            y_acc = list(state.bno_acc_norm)
            y_gyro = list(state.bno_gyro_norm)
            y_mag = list(state.bno_mag_norm)
            y_ang = list(state.bno_angle)
            y_sats = list(state.gps_sats)
            y_hdop = list(state.gps_hdop)
            y_detect = list(state.gps_detect)
            frame = state.last_frame
            age = time.time() - state.last_packet_time if state.last_packet_time else 999.0
            summary = dict(state.last_summary)

        line_acc.set_data(x, y_acc)
        line_gyro.set_data(x, y_gyro)
        line_mag.set_data(x, y_mag)
        line_ang.set_data(x, y_ang)
        line_sats.set_data(x, y_sats)
        line_hdop.set_data(x, y_hdop)
        line_detect.set_data(x, y_detect)

        xmin = max(0.0, (x[-1] - history_sec))
        xmax = max(history_sec, x[-1] + 0.1)
        for ax in axes.ravel():
            ax.set_xlim(xmin, xmax)
            ax.relim()
            ax.autoscale_view(scalex=False, scaley=True)

        dbg = summary.get("camera_debug", {}) or {}
        status = (
            f"link_age={age:.2f}s phase={summary.get('phase', -1)} | "
            f"cone_prob={summary.get('cone_probability', 0.0):.2f} cone_dir={summary.get('cone_direction', 0.5):.2f} "
            f"reached={summary.get('cone_is_reached', False)} goal_sign={dbg.get('goal_sign', False)} "
            f"bbox={dbg.get('bbox_px', None)} centroid={dbg.get('centroid_px', None)} "
            f"angle_valid={summary.get('angle_valid', False)} gps_detect={summary.get('gps_detect', 0)} "
            f"gps_heading_valid={summary.get('gps_heading_valid', False)} "
            f"lat={summary.get('gps_lat', 0.0):.6f} lng={summary.get('gps_lng', 0.0):.6f}"
        )
        status_text.set_text(status)

        if frame is not None:
            cv2.imshow("Camera Stream (SBC -> PC)", frame)
            cv2.waitKey(1)

        return line_acc, line_gyro, line_mag, line_ang, line_sats, line_hdop, line_detect, status_text

    ani = FuncAnimation(fig, update, interval=100, blit=False)
    plt.tight_layout()
    plt.show()
    _ = ani


def parse_args():
    parser = argparse.ArgumentParser(description="PC monitor for SBC camera/BNO055/GPS relay")
    parser.add_argument("--host", default="0.0.0.0", help="listen host")
    parser.add_argument("--port", type=int, default=5001, help="listen port")
    parser.add_argument("--history-sec", type=float, default=30.0, help="plot window seconds")
    return parser.parse_args()


def main():
    args = parse_args()
    state = MonitorState(history_sec=args.history_sec)
    server = RelayServer(args.host, args.port, state)

    th = threading.Thread(target=server.run, daemon=True)
    th.start()

    try:
        start_ui(state, history_sec=args.history_sec)
    finally:
        server.stop_event.set()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
