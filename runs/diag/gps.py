import sys
import os
import time
import argparse
from pathlib import Path

# Allow running this file directly (e.g. `python runs/gps_diag.py`) by adding
# the repository root to sys.path so `csmn` can be imported.
REPO_ROOT = Path(__file__).resolve().parents[2]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from csmn.gps_util import RobustGPSReader, open_gps_serial


UPDATE_INTERVAL_SECONDS = 5


def clear_screen():
    if os.name == "nt":
        os.system("cls")
    else:
        os.system("clear")


def _coordinate_display(fix, value_key, text_key):
    text_value = fix.get(text_key)
    if text_value:
        return text_value
    return f"{float(fix[value_key]):.8f}"


def read_raw_nmea(duration_seconds=10.0):
    print("=== RAW GPS INPUT PROBE ===")
    print(f"Duration: {duration_seconds}s")
    serial_obj, port, baud = open_gps_serial()
    if serial_obj is None:
        print("No GPS serial port produced NMEA output.")
        print("=== END RAW PROBE ===")
        return

    print(f"[Probe] {port} @ {baud}")
    try:
        start = time.time()
        total_bytes = 0
        lines_seen = 0
        while time.time() - start < duration_seconds:
            chunk = serial_obj.read(256)
            if not chunk:
                continue
            total_bytes += len(chunk)
            text = chunk.decode("utf-8", errors="ignore")
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
            serial_obj.close()
        except Exception:
            pass
    print("=== END RAW PROBE ===")


def parse_args():
    parser = argparse.ArgumentParser(description="GPS startup diagnostic tool")
    parser.add_argument("--raw-probe-seconds", type=float, default=10.0)
    parser.add_argument("--skip-raw-probe", action="store_true")
    parser.add_argument("--update-interval", type=float, default=UPDATE_INTERVAL_SECONDS)
    return parser.parse_args()


def main():
    args = parse_args()
    update_interval = max(0.5, float(args.update_interval))

    if not args.skip_raw_probe:
        read_raw_nmea(duration_seconds=max(0.0, float(args.raw_probe_seconds)))

    gps_reader = RobustGPSReader()
    last_print_time = 0.0
    first_fix_printed = False

    if args.skip_raw_probe:
        print("GPS monitor started without raw probe. This is the closest startup condition to mission code.")
    else:
        print("GPS monitor started after raw probe. Compare this against --skip-raw-probe to test startup warm-up effect.")
    print("GPS monitor (csmn.gps_util). Waiting for stable fix... Ctrl+C to exit.")
    try:
        while True:
            fix = gps_reader.read_fix()
            if fix is None:
                continue

            current_time = time.time()
            if current_time - last_print_time < update_interval:
                continue
            last_print_time = current_time

            if not first_fix_printed:
                clear_screen()
                first_fix_printed = True

            print("=" * 40)
            print("Stable GPS Fix Acquired")
            print(f"  Serial          : {fix.get('port')} @ {fix.get('baud')}")
            print(f"  Timestamp       : {fix.get('timestamp', '')}")
            print(f"  Latitude        : {_coordinate_display(fix, 'lat', 'lat_text')}")
            print(f"  Longitude       : {_coordinate_display(fix, 'lng', 'lng_text')}")
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
            print(f"({update_interval}秒ごとに更新。 Ctrl+Cで終了)")
            print("=" * 40)
    except KeyboardInterrupt:
        print("\nプログラムを終了します。")
    finally:
        if gps_reader.ser and gps_reader.ser.is_open:
            gps_reader.ser.close()


if __name__ == "__main__":
    main()
