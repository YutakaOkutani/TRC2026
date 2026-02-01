import csv
import datetime
import math
import os
import time
import traceback

import serial

from library import bno055
from library import bmp180
from library.micropyGPS import MicropyGPS

SERIAL_PORT = "/dev/serial0"
BAUD_RATE = 115200  # 9600, 38400
GPS_SERIAL_TIMEOUT = 1.0
GPS_TIMEZONE = 9
GPS_COORD_FORMAT = "dd"
GPS_BUFFER_CLEAR_THRESHOLD = 2048  # bytes; flush when backlog grows too large
GPS_BUFFER_CLEAR_INTERVAL = 5.0    # seconds between flush attempts
LOG_DIRECTORY = "testlog/open_parachute/"


def setup_sensors():
    """Initialize sensors and GPS serial connection."""
    bno = None
    bmp = None
    gps_serial = None

    print("Initializing BNO055...")
    try:
        bno = bno055.BNO055()
        if not bno.setUp():
            print("BNO055 setup returned False; disabling sensor.")
            bno = None
        else:
            print("BNO055 ready.")
    except Exception:
        print("Failed to initialize BNO055.")
        traceback.print_exc()
        bno = None

    print("Initializing BMP180...")
    try:
        bmp = bmp180.BMP180()
        if not bmp.setUp():
            print("BMP180 setup returned False; disabling sensor.")
            bmp = None
        else:
            temp = bmp.getTemperature()
            print(f"BMP180 ready. Current temperature: {temp:.2f} C")
    except Exception:
        print("Failed to initialize BMP180.")
        traceback.print_exc()
        bmp = None

    print("Opening GPS serial port...")
    try:
        gps_serial = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=GPS_SERIAL_TIMEOUT)
        gps_serial.reset_input_buffer()
        print("GPS serial ready.")
    except Exception as exc:
        print(f"Could not open GPS serial port {SERIAL_PORT}: {exc}")
        gps_serial = None

    return bno, bmp, gps_serial


def get_inertial_data(bno):
    """Read accelerometer, gyro, and magnetometer data."""
    data = {
        "acc_x": 0.0,
        "acc_y": 0.0,
        "acc_z": 0.0,
        "acc_combined": 0.0,
        "gyro_x": 0.0,
        "gyro_y": 0.0,
        "gyro_z": 0.0,
        "mag_x": 0.0,
        "mag_y": 0.0,
        "mag_z": 0.0,
    }

    if not bno:
        return data

    try:
        acc = bno.getAcc()
        data.update(
            {
                "acc_x": acc[0],
                "acc_y": acc[1],
                "acc_z": acc[2],
                "acc_combined": math.sqrt(acc[0] ** 2 + acc[1] ** 2 + acc[2] ** 2),
            }
        )
    except Exception:
        print("[BNO055] Failed to read acceleration.")

    try:
        gyro = bno.getGyro()
        data.update({"gyro_x": gyro[0], "gyro_y": gyro[1], "gyro_z": gyro[2]})
    except Exception:
        print("[BNO055] Failed to read gyro.")

    try:
        mag = bno.getMag()
        data.update({"mag_x": mag[0], "mag_y": mag[1], "mag_z": mag[2]})
    except Exception:
        print("[BNO055] Failed to read magnetometer.")

    return data


def get_environment_data(bmp):
    """Read temperature, pressure, and altitude from BMP180."""
    data = {"temp": 0.0, "pressure": 0.0, "altitude_bmp": 0.0}

    if not bmp:
        return data

    try:
        data.update(
            {
                "temp": bmp.getTemperature(),
                "pressure": bmp.getPressure(),
                "altitude_bmp": bmp.getAltitude(),
            }
        )
    except Exception:
        print("[BMP180] Failed to read data.")

    return data


def read_gps_data(gps_serial, gps_parser, last_buffer_clear):
    """Read GPS data using the same MicropyGPS flow as main.py."""
    if not gps_serial or gps_parser is None:
        return None, last_buffer_clear, gps_parser

    now = time.time()
    if (
        gps_serial.in_waiting > GPS_BUFFER_CLEAR_THRESHOLD
        and now - last_buffer_clear >= GPS_BUFFER_CLEAR_INTERVAL
    ):
        try:
            gps_serial.reset_input_buffer()
            gps_parser = MicropyGPS(GPS_TIMEZONE, GPS_COORD_FORMAT)
            print("GPS buffer cleared to drop stale data.")
        except Exception:
            pass
        last_buffer_clear = now

    try:
        line = gps_serial.readline().decode("utf-8", errors="ignore")
    except Exception:
        print("[GPS] Failed to read line.")
        return None, last_buffer_clear, gps_parser

    if len(line) > 0 and line[0] == "$":
        for ch in line:
            gps_parser.update(ch)

    lat = gps_parser.latitude[0]
    lng = gps_parser.longitude[0]
    if lat == 0.0:
        return None, last_buffer_clear, gps_parser

    altitude = getattr(gps_parser, "altitude", 0.0) or 0.0
    num_sats = getattr(gps_parser, "satellites_in_use", 0)
    ts = gps_parser.timestamp
    try:
        timestamp = f"{int(ts[0]):02d}:{int(ts[1]):02d}:{int(ts[2]):02d}"
    except Exception:
        timestamp = "00:00:00"

    return {
        "latitude": lat,
        "longitude": lng,
        "altitude_gps": altitude,
        "num_sats": num_sats,
        "gps_timestamp": timestamp,
    }, last_buffer_clear, gps_parser


def ensure_log_file(file_path, header):
    os.makedirs(LOG_DIRECTORY, exist_ok=True)
    with open(file_path, "w", newline="") as file:
        writer = csv.writer(file)
        writer.writerow(header)


def main():
    bno, bmp, gps_serial = setup_sensors()
    gps_parser = MicropyGPS(GPS_TIMEZONE, GPS_COORD_FORMAT)
    last_buffer_clear = time.time()

    if not any([bno, bmp, gps_serial]):
        print("All sensors failed to initialize; exiting.")
        return

    now_time = datetime.datetime.now()
    file_name = os.path.join(
        LOG_DIRECTORY, f"open_parachute_{now_time.strftime('%Y%m%d_%H%M%S')}.csv"
    )

    header = [
        "Time[s]",
        "Acc_X[m/s^2]",
        "Acc_Y[m/s^2]",
        "Acc_Z[m/s^2]",
        "Acc_Combined[m/s^2]",
        "Gyro_X[dps]",
        "Gyro_Y[dps]",
        "Gyro_Z[dps]",
        "Mag_X[uT]",
        "Mag_Y[uT]",
        "Mag_Z[uT]",
        "Temp[C]",
        "Pressure[hPa]",
        "Altitude_BMP[m]",
        "Latitude",
        "Longitude",
        "Altitude_GPS[m]",
        "Num_Satellites",
        "GPS_Timestamp",
    ]

    try:
        ensure_log_file(file_name, header)
    except IOError as exc:
        print(f"Failed to create log file: {exc}")
        return

    start_time = time.time()
    last_gps_data = {}
    loop_count = 0

    try:
        with open(file_name, "a", newline="") as file:
            writer = csv.writer(file)

            while True:
                loop_count += 1
                elapsed_time = time.time() - start_time

                inertial = get_inertial_data(bno)
                environment = get_environment_data(bmp)
                gps_data, last_buffer_clear, gps_parser = read_gps_data(
                    gps_serial, gps_parser, last_buffer_clear
                )

                if gps_data:
                    last_gps_data = gps_data

                merged = {**inertial, **environment, **last_gps_data}

                row_data = [
                    f"{elapsed_time:.3f}",
                    f"{merged.get('acc_x', 0.0):.4f}",
                    f"{merged.get('acc_y', 0.0):.4f}",
                    f"{merged.get('acc_z', 0.0):.4f}",
                    f"{merged.get('acc_combined', 0.0):.4f}",
                    f"{merged.get('gyro_x', 0.0):.4f}",
                    f"{merged.get('gyro_y', 0.0):.4f}",
                    f"{merged.get('gyro_z', 0.0):.4f}",
                    f"{merged.get('mag_x', 0.0):.4f}",
                    f"{merged.get('mag_y', 0.0):.4f}",
                    f"{merged.get('mag_z', 0.0):.4f}",
                    f"{merged.get('temp', 0.0):.2f}",
                    f"{merged.get('pressure', 0.0):.2f}",
                    f"{merged.get('altitude_bmp', 0.0):.2f}",
                    f"{merged.get('latitude', 0.0):.6f}",
                    f"{merged.get('longitude', 0.0):.6f}",
                    f"{merged.get('altitude_gps', 0.0)}",
                    merged.get("num_sats", 0),
                    merged.get("gps_timestamp", "00:00:00"),
                ]

                writer.writerow(row_data)
                file.flush()

                print(
                    f"--- Loop {loop_count} | Elapsed: {elapsed_time:.2f}s | "
                    f"Acc: {merged.get('acc_combined', 0.0):.3f} m/s^2 | "
                    f"Temp: {merged.get('temp', 0.0):.2f} C | "
                    f"GPS sats: {merged.get('num_sats', 0)}"
                )
    except KeyboardInterrupt:
        print("\nMeasurement stopped by user.")
    except Exception:
        print("Unexpected error during logging.")
        traceback.print_exc()
    finally:
        if gps_serial and gps_serial.is_open:
            gps_serial.close()
            print("Closed GPS serial port.")

        print(f"Log saved to {file_name}")


if __name__ == "__main__":
    main()
