import math
import sys
import time
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[2]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from lib import bno055, bmp180


def vec_norm(vec):
    values = [float(v) for v in vec[:3]]
    return math.sqrt(sum(v * v for v in values))


def main():
    print("Initializing BNO055...")
    bno = bno055.BNO055()
    bno_ok = bno.setUp()
    print(f"BNO055 setup: {bno_ok}")

    print("Initializing BMP180...")
    bmp = bmp180.BMP180(oss=3)
    bmp_ok = bmp.setUp()
    print(f"BMP180 setup: {bmp_ok}")

    print("Stationary BNO accel norm should be near 9.8 m/s^2.")
    print("Indoor BMP pressure should be near 90000-105000 Pa.")

    while True:
        now = time.strftime("%H:%M:%S")
        if bno_ok:
            acc = bno.getAcc()
            gyro = bno.getGyro()
            mag = bno.getMag()
            euler = bno.getEuler()
            sys_status = bno.getSystemStatus()
            sys_error = bno.getSystemError()
            calib = bno.getCalibrationStatus()
            if acc["valid"] and gyro["valid"] and mag["valid"] and euler["valid"]:
                print(
                    f"[{now}] BNO "
                    f"acc={acc['value']} |acc|={vec_norm(acc['value']):.3f} "
                    f"gyro={gyro['value']} |gyro|={vec_norm(gyro['value']):.3f} "
                    f"mag={mag['value']} |mag|={vec_norm(mag['value']):.3f} "
                    f"euler={euler['value']} sys={sys_status} err={sys_error} calib={calib}"
                )
            else:
                print(f"[{now}] BNO invalid packet acc={acc} gyro={gyro} mag={mag} euler={euler}")
        else:
            print(f"[{now}] BNO unavailable")

        if bmp_ok:
            temp = bmp.getTemperature()
            pres = bmp.getPressure()
            alt = 44330.0 * (1.0 - math.pow(float(pres) / 101325.0, 1.0 / 5.255))
            print(f"[{now}] BMP temp={float(temp):.2f}C pres={float(pres):.2f}Pa alt={alt:.2f}m")
        else:
            print(f"[{now}] BMP unavailable")

        time.sleep(1.0)


if __name__ == "__main__":
    main()
