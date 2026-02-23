import os
import time

from cansat_mission.gps_utils import RobustGPSReader, open_gps_serial


UPDATE_INTERVAL_SECONDS = 5


def clear_screen():
    if os.name == "nt":
        os.system("cls")
    else:
        os.system("clear")


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


def main():
    read_raw_nmea(duration_seconds=10.0)

    gps_reader = RobustGPSReader()
    last_print_time = 0.0
    first_fix_printed = False

    print("GPS monitor (cansat_mission.gps_utils). Waiting for stable fix... Ctrl+C to exit.")
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
            print("Stable GPS Fix Acquired")
            print(f"  Serial          : {fix.get('port')} @ {fix.get('baud')}")
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
