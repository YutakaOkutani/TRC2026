import csv
import datetime
import math
import os
import time
import traceback

import pynmea2
import serial

from library import BNO055 as bno055
from library import BMP180 as bmp180

SERIAL_PORT = "/dev/serial0"
BAUD_RATE = 115200
LOG_DIRECTORY = "/home/raspberry/log/landing_impact/"
GPS_READ_TIMEOUT = 0.5


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
        gps_serial = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1.0)
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


def parse_gga_line(line):
    """Parse a GGA sentence using pynmea2 and return a dict when valid."""
    try:
        message = pynmea2.parse(line)
    except pynmea2.ParseError:
        return None
    except Exception:
        print("[GPS] Unexpected parse error.")
        traceback.print_exc()
        return None

    if not isinstance(message, pynmea2.types.talker.GGA):
        return None

    # gps_qual == 0 means invalid fix
    if getattr(message, "gps_qual", 0) in (None, 0):
        return None

    if not message.latitude or not message.longitude:
        return None

    timestamp = "00:00:00"
    try:
        if message.timestamp:
            timestamp = message.timestamp.strftime("%H:%M:%S")
    except Exception:
        timestamp = "00:00:00"

    try:
        altitude = float(message.altitude) if message.altitude else 0.0
    except Exception:
        altitude = 0.0

    try:
        num_sats = int(message.num_sats) if message.num_sats else 0
    except Exception:
        num_sats = 0

    return {
        "latitude": message.latitude,
        "longitude": message.longitude,
        "altitude_gps": altitude,
        "num_sats": num_sats,
        "gps_timestamp": timestamp,
    }


def read_gps_fix(gps_serial, timeout=GPS_READ_TIMEOUT):
    """Read GPS data for up to `timeout` seconds and return the first valid fix."""
    if not gps_serial:
        return None

    deadline = time.time() + timeout
    while time.time() < deadline:
        try:
            line = gps_serial.readline().decode("utf-8", errors="ignore")
        except Exception:
            print("[GPS] Failed to read line.")
            return None

        if not line.startswith(("$GPGGA", "$GNGGA")):
            continue

        gps_data = parse_gga_line(line)
        if gps_data:
            return gps_data

    return None


def ensure_log_file(file_path, header):
    os.makedirs(LOG_DIRECTORY, exist_ok=True)
    with open(file_path, "w", newline="") as file:
        writer = csv.writer(file)
        writer.writerow(header)


def main():
    bno, bmp, gps_serial = setup_sensors()
    if not any([bno, bmp, gps_serial]):
        print("All sensors failed to initialize; exiting.")
        return

    now_time = datetime.datetime.now()
    file_name = os.path.join(
        LOG_DIRECTORY, f"impact_log_{now_time.strftime('%Y%m%d_%H%M%S')}.csv"
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
                gps_data = read_gps_fix(gps_serial)

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
