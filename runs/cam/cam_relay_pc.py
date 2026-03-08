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

# Runtime defaults (change here when your environment changes)
# Variable parameter: PC monitor bind address
DEFAULT_MONITOR_HOST = "0.0.0.0"
# Variable parameter: PC monitor listen port
DEFAULT_MONITOR_PORT = 5001
# Variable parameter: graph history window [sec]
DEFAULT_HISTORY_SEC = 30.0
CAMERA_WINDOW_NAME = "Camera Stream (SBC -> PC)"


def get_screen_size():
    try:
        import tkinter as tk

        root = tk.Tk()
        root.withdraw()
        width = int(root.winfo_screenwidth())
        height = int(root.winfo_screenheight())
        root.destroy()
        if width > 0 and height > 0:
            return width, height
    except Exception:
        pass
    return 1920, 1080


def fit_frame_to_screen(frame, screen_size):
    if frame is None:
        return None
    screen_w, screen_h = screen_size
    if screen_w <= 0 or screen_h <= 0:
        return frame

    h, w = frame.shape[:2]
    if h <= 0 or w <= 0:
        return frame

    scale = min(float(screen_w) / float(w), float(screen_h) / float(h))
    out_w = max(1, int(round(w * scale)))
    out_h = max(1, int(round(h * scale)))
    resized = cv2.resize(frame, (out_w, out_h), interpolation=cv2.INTER_LINEAR)
    canvas = np.zeros((screen_h, screen_w, 3), dtype=np.uint8)
    x0 = max(0, (screen_w - out_w) // 2)
    y0 = max(0, (screen_h - out_h) // 2)
    canvas[y0 : y0 + out_h, x0 : x0 + out_w] = resized
    return canvas


def setup_camera_window(fullscreen):
    cv2.namedWindow(CAMERA_WINDOW_NAME, cv2.WINDOW_NORMAL)
    if fullscreen:
        try:
            cv2.setWindowProperty(CAMERA_WINDOW_NAME, cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)
        except Exception:
            pass


def recv_exact(sock, n):
    data = bytearray()
    while len(data) < n:
        try:
            chunk = sock.recv(n - len(data))
        except (socket.timeout, TimeoutError):
            return None
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
        self.shutdown_requested = False
        self.shutdown_reason = None

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
                "camera_method": packet.get("camera", {}).get("debug", {}).get("method", ""),
                "angle_valid": bno.get("angle_valid", False),
                "gps_detect": gps.get("gps_detect", 0),
                "gps_heading_valid": gps.get("gps_heading_valid", False),
                "gps_lat": gps.get("lat", 0.0),
                "gps_lng": gps.get("lng", 0.0),
            }

    def request_shutdown(self, reason):
        with self.lock:
            self.shutdown_requested = True
            self.shutdown_reason = reason or "SBC_SHUTDOWN"


class RelayServer:
    def __init__(self, host, port, state):
        self.host = host
        self.port = port
        self.st = state
        self.stop_event = threading.Event()
        self._server_sock = None
        self._conn_sock = None
        self._sock_lock = threading.Lock()

    def shutdown(self):
        self.stop_event.set()
        with self._sock_lock:
            socks = [self._conn_sock, self._server_sock]
            self._conn_sock = None
            self._server_sock = None
        for sock in socks:
            if sock is None:
                continue
            try:
                sock.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass
            try:
                sock.close()
            except OSError:
                pass

    def run(self):
        server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server.settimeout(0.2)
        server.bind((self.host, self.port))
        server.listen(1)
        with self._sock_lock:
            self._server_sock = server
        print(f"Listening on {self.host}:{self.port}")

        try:
            while not self.stop_event.is_set():
                try:
                    conn, addr = server.accept()
                except (socket.timeout, TimeoutError):
                    continue
                except OSError:
                    if self.stop_event.is_set():
                        break
                    raise
                print(f"Connected: {addr[0]}:{addr[1]}")
                with conn:
                    conn.settimeout(0.2)
                    with self._sock_lock:
                        self._conn_sock = conn
                    try:
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
                                self.st.update(packet)
                            elif packet.get("type") == "shutdown":
                                reason = packet.get("reason", "SBC_SHUTDOWN")
                                print(f"Received shutdown packet from SBC (reason={reason})")
                                self.st.request_shutdown(reason)
                                self.stop_event.set()
                                break
                    except (socket.timeout, TimeoutError) as exc:
                        print(f"Connection timeout: {exc}")
                    finally:
                        with self._sock_lock:
                            if self._conn_sock is conn:
                                self._conn_sock = None
                print("Disconnected. Waiting for reconnect...")
        finally:
            with self._sock_lock:
                if self._server_sock is server:
                    self._server_sock = None
            server.close()


def start_ui(state, history_sec, camera_fullscreen=True):
    fig, axes = plt.subplots(2, 2, figsize=(10, 6))
    fig.suptitle("CanSat Camera/BNO055/GPS Realtime Monitor")
    ui_state = {"window_closed": False}
    screen_size = get_screen_size()
    setup_camera_window(camera_fullscreen)

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

    def _on_close(_event):
        ui_state["window_closed"] = True

    fig.canvas.mpl_connect("close_event", _on_close)

    def update(_):
        with state.lock:
            shutdown_requested = state.shutdown_requested
            shutdown_reason = state.shutdown_reason
            if len(state.t) == 0:
                if shutdown_requested:
                    status_text.set_text(f"shutdown requested: {shutdown_reason}")
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
            f"method={summary.get('camera_method', '')} "
            f"bbox={dbg.get('bbox_px', None)} centroid={dbg.get('centroid_px', None)} "
            f"angle_valid={summary.get('angle_valid', False)} gps_detect={summary.get('gps_detect', 0)} "
            f"gps_heading_valid={summary.get('gps_heading_valid', False)} "
            f"lat={summary.get('gps_lat', 0.0):.6f} lng={summary.get('gps_lng', 0.0):.6f}"
        )
        status_text.set_text(status)

        if shutdown_requested:
            status_text.set_text(f"{status} | shutdown requested: {shutdown_reason}")

        if frame is not None:
            camera_view = fit_frame_to_screen(frame, screen_size) if camera_fullscreen else frame
            cv2.imshow(CAMERA_WINDOW_NAME, camera_view)
            cv2.waitKey(1)

        return line_acc, line_gyro, line_mag, line_ang, line_sats, line_hdop, line_detect, status_text

    ani = FuncAnimation(fig, update, interval=100, blit=False, cache_frame_data=False)
    plt.tight_layout()
    plt.show(block=False)

    try:
        while plt.fignum_exists(fig.number):
            with state.lock:
                if state.shutdown_requested:
                    print(f"PC monitor exiting (SBC reason={state.shutdown_reason})")
                    break
            plt.pause(0.05)
    finally:
        try:
            if getattr(ani, "event_source", None) is not None:
                ani.event_source.stop()
        except Exception:
            pass
        if not ui_state["window_closed"] and plt.fignum_exists(fig.number):
            try:
                plt.close(fig)
            except Exception:
                pass
        _ = ani


def parse_args():
    parser = argparse.ArgumentParser(description="PC monitor for SBC camera/BNO055/GPS relay")
    parser.add_argument("--host", default=DEFAULT_MONITOR_HOST, help="listen host")
    parser.add_argument("--port", type=int, default=DEFAULT_MONITOR_PORT, help="listen port")
    parser.add_argument("--history-sec", type=float, default=DEFAULT_HISTORY_SEC, help="plot window seconds")
    parser.add_argument(
        "--camera-fullscreen",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="show the camera preview in a fullscreen OpenCV window",
    )
    return parser.parse_args()


def main():
    args = parse_args()
    state = MonitorState(history_sec=args.history_sec)
    server = RelayServer(args.host, args.port, state)

    th = threading.Thread(target=server.run, daemon=True)
    th.start()

    try:
        start_ui(state, history_sec=args.history_sec, camera_fullscreen=args.camera_fullscreen)
    except KeyboardInterrupt:
        print("Interrupted by Ctrl+C")
    finally:
        server.shutdown()
        cv2.destroyAllWindows()
        th.join(timeout=1.0)


if __name__ == "__main__":
    main()
