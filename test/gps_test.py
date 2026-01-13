import serial
import time
import os


# --- 設定 ---
SERIAL_PORT = "/dev/serial0"   # 使用しているポートに合わせて変更
BAUD_RATE = 115200              # GPSモジュールのボーレートに合わせて変更（例：9600）
UPDATE_INTERVAL_SECONDS = 5    # 何秒ごとに表示を更新するか


def clear_screen():
    """画面をクリアする関数"""
    if os.name == "nt":  # Windows
        os.system("cls")
    else:                # Linux / macOS
        os.system("clear")


def nmea_to_decimal_degrees(nmea_value, direction):
    """
    NMEA形式の緯度経度を10進数度に変換する関数。
    例: '3530.1234', 'N' -> 35.502056...
        '13944.5678', 'E' -> 139.742797...
    """
    if not nmea_value or nmea_value == "0":
        return 0.0

    try:
        # 緯度: ddmm.mmmm, 経度: dddmm.mmmm という形式
        if "." not in nmea_value:
            return 0.0

        # 小数点より前を取得して度と分に分割
        head, tail = nmea_value.split(".")
        if len(head) <= 2:
            # 想定外フォーマット
            return 0.0

        # 緯度: 2桁が度, 経度: 3桁が度
        if len(head) == 4:      # 例: 3530 -> 35度30分
            deg = int(head[:2])
            minutes = float(head[2:] + "." + tail)
        else:                   # 5桁以上は経度とみなす 例: 13944 -> 139度44分
            deg = int(head[:-2])
            minutes = float(head[-2:] + "." + tail)

        decimal_deg = deg + minutes / 60.0

        if direction in ("S", "W"):
            decimal_deg *= -1

        return decimal_deg
    except (ValueError, IndexError):
        return 0.0


def get_fix_quality_label(gps_qual_value):
    """
    GGAセンテンスの gps_qual から説明文字列を返す。
    """
    try:
        q = int(gps_qual_value)
    except (TypeError, ValueError):
        return "不明"

    mapping = {
        0: "No Fix（未測位）",
        1: "GPS Fix（標準測位）",
        2: "DGPS Fix（補強あり）",
        4: "RTK Fixed",
        5: "RTK Float",
    }
    return mapping.get(q, "不明")


def parse_gga(sentence):
    """
    GGAセンテンスを分解して必要な項目を辞書で返す。
    対応するフィールド：
    0: $GPGGA
    1: 時刻 (hhmmss.sss)
    2: 緯度 (ddmm.mmmm)
    3: 北緯/南緯 (N/S)
    4: 経度 (dddmm.mmmm)
    5: 東経/西経 (E/W)
    6: Fix品質 (0～)
    7: 使用衛星数
    8: HDOP
    9: 高度
    10: 高度の単位 (M)
    """
    parts = sentence.strip().split(",")

    # 最低限の長さチェック
    if len(parts) < 11:
        return None

    if not (parts[0].startswith("$GPGGA") or parts[0].startswith("$GNGGA")):
        return None

    time_str = parts[1]
    lat_raw = parts[2]
    lat_dir = parts[3]
    lon_raw = parts[4]
    lon_dir = parts[5]
    gps_qual = parts[6]
    num_sats = parts[7]
    hdop = parts[8]
    altitude = parts[9]
    altitude_units = parts[10]

    # 緯度経度を10進数へ変換
    lat = nmea_to_decimal_degrees(lat_raw, lat_dir)
    lon = nmea_to_decimal_degrees(lon_raw, lon_dir)

    # HDOP を数値に変換
    hdop_value = None
    if hdop not in ("", None):
        try:
            hdop_value = float(hdop)
        except ValueError:
            hdop_value = None

    return {
        "time_str": time_str,
        "lat": lat,
        "lat_dir": lat_dir,
        "lon": lon,
        "lon_dir": lon_dir,
        "gps_qual": gps_qual,
        "num_sats": num_sats,
        "hdop_raw": hdop,
        "hdop_value": hdop_value,
        "altitude": altitude,
        "altitude_units": altitude_units,
    }


def main():
    try:
        ser = serial.Serial(SERIAL_PORT, baudrate=BAUD_RATE, timeout=1)
    except serial.SerialException as e:
        print(f"シリアルポートを開けませんでした: {e}")
        return

    print("🛰️ GPS受信待機中... (Ctrl+C で終了)")
    last_print_time = 0.0
    is_first_fix = True

    try:
        while True:
            line = ser.readline().decode("utf-8", errors="ignore")

            if not line.startswith(("$GPGGA", "$GNGGA")):
                continue

            data = parse_gga(line)
            if data is None:
                continue

            current_time = time.time()
            if current_time - last_print_time < UPDATE_INTERVAL_SECONDS:
                continue

            lat_valid = data["lat"] not in (None, 0.0)
            lon_valid = data["lon"] not in (None, 0.0)

            gps_qual_raw = data["gps_qual"]
            fix_quality_label = get_fix_quality_label(gps_qual_raw)
            hdop_value = data["hdop_value"]
            hdop_raw = data["hdop_raw"]
            num_sats = data["num_sats"]
            altitude_raw = data["altitude"]
            altitude_units = data["altitude_units"]

            # 最初の「有効な測位」で画面クリア
            if lat_valid and lon_valid and is_first_fix:
                clear_screen()
                is_first_fix = False

            print("=" * 40)
            if lat_valid and lon_valid:
                print("✅ GPS測位成功")
            else:
                print("⚠️ まだ有効なGPS測位が取れていない可能性があります")

            print("-" * 40)
            print(f"  タイムスタンプ : {data['time_str']}")
            print(f"  Fix品質        : {gps_qual_raw} ({fix_quality_label})")

            if hdop_value is not None:
                print(f"  HDOP           : {hdop_value:.2f}")
            elif hdop_raw not in (None, ""):
                print(f"  HDOP           : {hdop_raw} (数値変換不可)")

            # 位置情報
            if lat_valid and lon_valid:
                print(f"  緯度           : {data['lat']:.6f} {data['lat_dir']}")
                print(f"  経度           : {data['lon']:.6f} {data['lon_dir']}")
            else:
                print("  緯度・経度     : 無効（0 または 未設定）")

            # 高度
            if altitude_raw not in (None, ""):
                print(f"  高度           : {altitude_raw} {altitude_units}")
            else:
                print("  高度           : 不明")

            print(f"  使用衛星数     : {num_sats}")
            print("-" * 40)

            # 簡易警告
            warnings = []

            # 衛星数
            try:
                num_sats_int = int(num_sats)
                if num_sats_int < 4:
                    warnings.append("衛星数が 4 未満のため、3D測位が不安定な可能性があります。")
            except (TypeError, ValueError):
                if num_sats not in ("", None):
                    warnings.append(f"衛星数が数値として解釈できません: {num_sats}")

            # HDOP
            if hdop_value is not None:
                if hdop_value > 5.0:
                    warnings.append("HDOP が大きく、位置精度が悪い可能性があります。")
                elif hdop_value > 2.5:
                    warnings.append("HDOP がやや大きめです（中程度の精度）。")

            # 高度のざっくりチェック
            try:
                if altitude_raw not in (None, ""):
                    alt_val = float(altitude_raw)
                    if alt_val < -100 or alt_val > 10000:
                        warnings.append("高度が異常値の可能性があります（-100〜10000mの範囲外）。")
            except ValueError:
                warnings.append(f"高度が数値として解釈できません: {altitude_raw}")

            # 緯度経度が無効
            if not lat_valid or not lon_valid:
                warnings.append("緯度または経度が 0 もしくは未設定です。まだ Fix が完了していない可能性があります。")

            if warnings:
                print("⚠️ 注意 / コメント")
                for w in warnings:
                    print(f"   - {w}")
                print("-" * 40)

            print(f"({UPDATE_INTERVAL_SECONDS}秒ごとに更新。 Ctrl+Cで終了)")
            print("=" * 40)

            last_print_time = current_time

    except KeyboardInterrupt:
        print("\nプログラムを終了します。")
    finally:
        if ser.is_open:
            ser.close()


if __name__ == "__main__":
    main()
