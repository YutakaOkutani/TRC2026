import sys
import time

import serial

from library.micropyGPS import MicropyGPS

# Match main.py defaults
GPS_SERIAL_PORT = "/dev/serial0"
GPS_BAUDRATE = 115200 # 9600, 38400
GPS_SERIAL_TIMEOUT = 1
GPS_TIMEZONE = 9
GPS_COORD_FORMAT = "dd"
GPS_BUFFER_CLEAR_THRESHOLD = 2048
GPS_BUFFER_CLEAR_INTERVAL = 5.0
REPORT_INTERVAL = 1.5


def open_serial(port, baud, timeout):
    try:
        ser = serial.Serial(port, baudrate=baud, timeout=timeout)
        try:
            ser.reset_input_buffer()
        except Exception:
            pass
        return ser
    except serial.SerialException as exc:
        print(f"Failed to open serial port {port}: {exc}")
        return None


def feed_sentence(gps, sentence):
    last = None
    for ch in sentence:
        parsed = gps.update(ch)
        if parsed:
            last = parsed
    return last


def format_gps_time(gps):
    try:
        hh, mm, ss = gps.timestamp
        return f"{int(hh):02d}:{int(mm):02d}:{float(ss):05.2f}"
    except Exception:
        return "--:--:--"


def format_gps_date(gps):
    try:
        day, month, year = gps.date
        return f"20{int(year):02d}-{int(month):02d}-{int(day):02d}"
    except Exception:
        return "----/--/--"


def print_status(gps, last_sentence):
    lat_data = gps.latitude
    lon_data = gps.longitude
    lat_val = lat_data[0] if len(lat_data) > 0 else 0.0
    lon_val = lon_data[0] if len(lon_data) > 0 else 0.0
    lat_hemi = lat_data[1] if len(lat_data) > 1 else ""
    lon_hemi = lon_data[1] if len(lon_data) > 1 else ""

    has_fix = gps.valid and lat_val != 0.0 and lon_val != 0.0
    fix_quality = gps.fix_stat  # from GGA
    fix_type = gps.fix_type     # from GSA

    status_label = "FIX" if has_fix else "SEARCH"
    fix_quality_label = {0: "invalid", 1: "GPS", 2: "DGPS"}.get(fix_quality, "?")
    fix_type_label = {1: "no fix", 2: "2D", 3: "3D"}.get(fix_type, "?")

    timestamp = format_gps_time(gps)
    date_str = format_gps_date(gps)

    print("-" * 60)
    print(f"[{time.strftime('%H:%M:%S')}] Sent:{last_sentence or '-'} Status:{status_label}")
    print(f"UTC {timestamp} Date {date_str} | Sentences clean:{gps.clean_sentences} crc_fail:{gps.crc_fails}")
    print(f"Lat {lat_val:.6f} {lat_hemi}  Lon {lon_val:.6f} {lon_hemi}")
    print(f"Alt {gps.altitude:.2f} m  Geoid {gps.geoid_height:.2f} m")
    print(f"Speed {gps.speed[2]:.2f} km/h  Course {gps.course:.2f} deg")
    print(
        f"Sats use/view {gps.satellites_in_use}/{gps.satellites_in_view} "
        f"HDOP {gps.hdop:.2f} PDOP {gps.pdop:.2f} VDOP {gps.vdop:.2f}"
    )
    print(f"Fix quality (GGA): {fix_quality} [{fix_quality_label}]  Fix type (GSA): {fix_type} [{fix_type_label}]")


def main():
    gps = MicropyGPS(GPS_TIMEZONE, GPS_COORD_FORMAT)
    ser = open_serial(GPS_SERIAL_PORT, GPS_BAUDRATE, GPS_SERIAL_TIMEOUT)
    last_report = 0.0
    last_buffer_clear = time.time()
    last_sentence = None

    if ser is None:
        print("Waiting for serial port... (retrying)")

    try:
        while True:
            if ser is None or not ser.is_open:
                time.sleep(1.0)
                ser = open_serial(GPS_SERIAL_PORT, GPS_BAUDRATE, GPS_SERIAL_TIMEOUT)
                continue

            now = time.time()
            if (
                ser.in_waiting > GPS_BUFFER_CLEAR_THRESHOLD
                and now - last_buffer_clear >= GPS_BUFFER_CLEAR_INTERVAL
            ):
                try:
                    ser.reset_input_buffer()
                    gps = MicropyGPS(GPS_TIMEZONE, GPS_COORD_FORMAT)
                    last_sentence = None
                    print("GPS buffer cleared to drop stale data.")
                except Exception:
                    pass
                last_buffer_clear = now

            try:
                line = ser.readline().decode("utf-8", errors="ignore")
            except Exception as exc:
                print(f"Read error: {exc}")
                time.sleep(0.5)
                continue

            if line.startswith("$"):
                parsed_type = feed_sentence(gps, line)
                if parsed_type:
                    last_sentence = parsed_type

            if now - last_report >= REPORT_INTERVAL:
                print_status(gps, last_sentence)
                last_report = now

    except KeyboardInterrupt:
        print("\nStop requested. Closing serial port.")
    finally:
        if ser and ser.is_open:
            ser.close()


if __name__ == "__main__":
    sys.exit(main())
