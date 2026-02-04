import math
import os
import pynmea2
import time

import serial


def configure_uart_pins():
    """Ensure GPIO14/15 are mapped to UART (TX/RX) using gpiozero."""
    try:
        from gpiozero.pins.lgpio import LGPIOFactory
    except Exception:
        return
    try:
        pin_factory = LGPIOFactory()
        for pin_no in (14, 15):
            pin = pin_factory.pin(pin_no)
            try:
                pin.function = "alt0"  # UART0: TXD0/RXD0
            finally:
                try:
                    pin.close()
                except Exception:
                    pass
    except Exception:
        return

# --- GPS settings (identical to main.py) ---
GPS_SERIAL_PORT = "/dev/serial0"
GPS_SERIAL_PORT_CANDIDATES = ["/dev/serial0", "/dev/ttyAMA0", "/dev/ttyS0"]
GPS_BAUDRATE = 115200  # 9600, 38400
GPS_BAUDRATE_CANDIDATES = [115200, 9600, 38400]
GPS_SERIAL_TIMEOUT = 1
GPS_BUFFER_CLEAR_THRESHOLD = 2048  # bytes; flush when backlog grows too large
GPS_BUFFER_CLEAR_INTERVAL = 5.0    # seconds between flush attempts
GPS_MIN_FIX_QUAL = 1              # 1: GPS fix, 2: DGPS, 4/5: RTK
GPS_MIN_SATELLITES = 4
GPS_MAX_HDOP = 5.0
GPS_MAX_SPEED_MPS = 50.0          # reject if jump implies speed over this (m/s)
GPS_STABLE_FIX_COUNT = 3          # consecutive good fixes required
GPS_FIX_LOSS_TIMEOUT = 8.0        # seconds until detect flag drops when no valid fix

# --- Display update interval ---
UPDATE_INTERVAL_SECONDS = 5


def clear_screen():
    """Clear terminal screen for readability."""
    if os.name == "nt":
        os.system("cls")
    else:
        os.system("clear")


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


def read_raw_nmea(duration_seconds=10.0):
    """Read any bytes from candidate ports to verify physical connection."""
    configure_uart_pins()
    ports = [GPS_SERIAL_PORT] + [p for p in GPS_SERIAL_PORT_CANDIDATES if p != GPS_SERIAL_PORT]
    bauds = [GPS_BAUDRATE] + [b for b in GPS_BAUDRATE_CANDIDATES if b != GPS_BAUDRATE]
    print("=== RAW GPS INPUT PROBE ===")
    print(f"Duration: {duration_seconds}s")
    print(f"Ports: {ports}")
    print(f"Bauds: {bauds}")
    for port in ports:
        for baud in bauds:
            print(f"\n[Probe] {port} @ {baud}")
            try:
                ser = serial.Serial(port, baud, timeout=0.5)
            except Exception as e:
                print(f"  Open failed: {e}")
                continue
            try:
                try:
                    ser.reset_input_buffer()
                except Exception:
                    pass
                start = time.time()
                total_bytes = 0
                lines_seen = 0
                while time.time() - start < duration_seconds:
                    chunk = ser.read(256)
                    if not chunk:
                        continue
                    total_bytes += len(chunk)
                    try:
                        text = chunk.decode("utf-8", errors="ignore")
                    except Exception:
                        text = repr(chunk)
                    if "$" in text:
                        lines_seen += text.count("$")
                    print(text, end="")
                print("")
                if total_bytes == 0:
                    print("  No bytes received.")
                else:
                    print(f"  Bytes received: {total_bytes} (approx NMEA lines: {lines_seen})")
            finally:
                try:
                    ser.close()
                except Exception:
                    pass
    print("=== END RAW PROBE ===")


class RobustGPSReader:
    """Replicates the GPS handling logic from main.py for standalone scripts."""

    def __init__(self):
        self.ser = None
        self.last_buffer_clear = time.time()
        self.last_fix_time = 0.0
        self.last_valid_fix_time = 0.0
        self.last_valid_latlng = None
        self.stable_count = 0

    def _probe_nmea(self, ser, probe_seconds=2.0):
        start = time.time()
        last_line = ""
        while time.time() - start < probe_seconds:
            try:
                line_bytes = ser.readline()
            except Exception:
                return False, last_line
            if not line_bytes:
                continue
            line = line_bytes.decode("utf-8", errors="ignore").strip()
            if line:
                last_line = line
            if line.startswith("$"):
                return True, last_line
        return False, last_line

    def open_serial(self):
        configure_uart_pins()
        ports = [GPS_SERIAL_PORT] + [p for p in GPS_SERIAL_PORT_CANDIDATES if p != GPS_SERIAL_PORT]
        bauds = [GPS_BAUDRATE] + [b for b in GPS_BAUDRATE_CANDIDATES if b != GPS_BAUDRATE]
        for port in ports:
            for baud in bauds:
                try:
                    ser = serial.Serial(port, baud, timeout=0.2)
                    try:
                        ser.reset_input_buffer()
                    except Exception:
                        pass
                    ok, last_line = self._probe_nmea(ser)
                    if ok:
                        ser.timeout = GPS_SERIAL_TIMEOUT
                        print(f"GPS serial opened: {port} @ {baud}")
                        return ser
                    try:
                        ser.close()
                    except Exception:
                        pass
                    if last_line:
                        print(f"No NMEA at {port} @ {baud} (last: {last_line})")
                    else:
                        print(f"No NMEA at {port} @ {baud}")
                except Exception as e:
                    print(f"GPS Serial Open Failed: {port} @ {baud}: {e}")
        return None

    def ensure_serial(self):
        if self.ser is None or not self.ser.is_open:
            self.ser = self.open_serial()

    def read_fix(self):
        """Return a stable fix dict or None, using the same gates as main.py."""
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
        except Exception as e:
            print(f"GPS read error: {e}")
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
            try:
                timestamp = msg.timestamp.strftime("%H:%M:%S")
            except Exception:
                timestamp = str(getattr(msg, "timestamp", ""))
            try:
                altitude = float(getattr(msg, "altitude", 0.0) or 0.0)
            except Exception:
                altitude = 0.0
            return {
                "lat": lat,
                "lng": lng,
                "gps_qual": gps_qual,
                "num_sats": num_sats,
                "hdop": hdop,
                "altitude": altitude,
                "timestamp": timestamp,
                "raw": line,
            }

        return None


def main():
    # Raw probe first for physical-connection diagnostics.
    read_raw_nmea(duration_seconds=10.0)

    gps_reader = RobustGPSReader()
    last_print_time = 0.0
    first_fix_printed = False

    print("🛰️ GPS monitor (main.py logic). Waiting for stable fix... Ctrl+C to exit.")
    try:
        while True:
            fix = gps_reader.read_fix()
            if fix is None:
                continue

            current_time = time.time()
            if current_time - last_print_time < UPDATE_INTERVAL_SECONDS:
                continue
            last_print_time = current_time

            if not first_fix_printed:
                clear_screen()
                first_fix_printed = True

            print("=" * 40)
            print("✅ Stable GPS Fix Acquired")
            print(f"  Timestamp       : {fix.get('timestamp', '')}")
            print(f"  Latitude        : {fix['lat']:.6f}")
            print(f"  Longitude       : {fix['lng']:.6f}")
            print(f"  Altitude (GGA)  : {fix['altitude']} m")
            print(f"  Fix Quality     : {fix.get('gps_qual')}")
            print(f"  Satellites      : {fix.get('num_sats')}")
            hdop = fix.get("hdop")
            if hdop is not None:
                try:
                    print(f"  HDOP            : {float(hdop):.2f}")
                except Exception:
                    print(f"  HDOP            : {hdop}")
            print("-" * 40)
            print(f"Raw: {fix.get('raw', '')}")
            print(f"({UPDATE_INTERVAL_SECONDS}秒ごとに更新。 Ctrl+Cで終了)")
            print("=" * 40)
    except KeyboardInterrupt:
        print("\nプログラムを終了します。")
    finally:
        if gps_reader.ser and gps_reader.ser.is_open:
            gps_reader.ser.close()


if __name__ == "__main__":
    main()
