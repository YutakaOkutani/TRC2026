import serial
import pynmea2
import time
import os


def clear_screen():
    """画面をクリアする関数"""
    # Windowsの場合
    if os.name == "nt":
        _ = os.system("cls")
    # MacやLinuxの場合
    else:
        _ = os.system("clear")


def get_fix_quality_label(gps_qual_value):
    """
    GGAセンテンスの gps_qual から、人間向けの説明文を返す。
    gps_qual は pynmea2 では文字列として渡ってくる場合がある。
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


# --- 設定 ---
SERIAL_PORT = "/dev/serial0"
BAUD_RATE = 115200 # GPSモジュールに合わせて設定（例：9600）
UPDATE_INTERVAL_SECONDS = 5  # 表示を更新する間隔（秒）


try:
    ser = serial.Serial(SERIAL_PORT, baudrate=BAUD_RATE, timeout=1)
    print("🛰️ GPS受信待機中... (Ctrl+C で終了)")
    last_print_time = 0.0
    is_first_fix = True

    while True:
        line = ser.readline().decode("utf-8", errors="ignore")

        # GGAセンテンス（位置情報）の場合のみ処理
        if line.startswith(("$GPGGA", "$GNGGA")):
            try:
                msg = pynmea2.parse(line)

                current_time = time.time()
                # 一定間隔ごとにのみ表示
                if current_time - last_print_time < UPDATE_INTERVAL_SECONDS:
                    continue

                # 緯度・経度が有効かどうかを判定
                lat_valid = msg.latitude not in (None, 0.0)
                lon_valid = msg.longitude not in (None, 0.0)

                # Fix品質・HDOPなどを取得
                gps_qual_raw = getattr(msg, "gps_qual", None)
                fix_quality_label = get_fix_quality_label(gps_qual_raw)

                hdop_raw = getattr(msg, "horizontal_dil", None)
                hdop_value = None
                if hdop_raw not in (None, ""):
                    try:
                        hdop_value = float(hdop_raw)
                    except ValueError:
                        hdop_value = None

                num_sats = getattr(msg, "num_sats", "")

                # 最初の有効な測位時に画面をクリア
                if lat_valid and lon_valid and is_first_fix:
                    clear_screen()
                    is_first_fix = False

                # 共通ヘッダ
                print("=" * 40)
                if lat_valid and lon_valid:
                    print("✅ GPS測位成功")
                else:
                    print("⚠️ まだ有効なGPS測位が取れていない可能性があります")

                print("-" * 40)
                print(f"  タイムスタンプ : {msg.timestamp}")
                print(f"  Fix品質        : {gps_qual_raw} ({fix_quality_label})")

                if hdop_value is not None:
                    print(f"  HDOP           : {hdop_value:.2f}")
                elif hdop_raw not in (None, ""):
                    # 数値変換できなかった場合も念のため表示
                    print(f"  HDOP           : {hdop_raw} (数値変換不可)")

                # 位置情報
                if lat_valid and lon_valid:
                    print(f"  緯度           : {msg.latitude:.6f} {msg.lat_dir}")
                    print(f"  経度           : {msg.longitude:.6f} {msg.lon_dir}")
                else:
                    print("  緯度・経度     : 無効（0 または 未設定）")

                # 高度
                altitude_raw = getattr(msg, "altitude", None)
                altitude_units = getattr(msg, "altitude_units", "")
                if altitude_raw not in (None, ""):
                    print(f"  高度           : {altitude_raw} {altitude_units}")
                else:
                    print("  高度           : 不明")

                print(f"  使用衛星数     : {num_sats}")
                print("-" * 40)

                # 簡易チェック・警告表示
                warnings = []

                # 衛星数に関する簡易チェック
                try:
                    num_sats_int = int(num_sats)
                    if num_sats_int < 4:
                        warnings.append("衛星数が 4 未満のため、3D測位が不安定な可能性があります。")
                except (TypeError, ValueError):
                    if num_sats not in ("", None):
                        warnings.append(f"衛星数が数値として解釈できません: {num_sats}")

                # HDOP による精度評価
                if hdop_value is not None:
                    if hdop_value > 5.0:
                        warnings.append("HDOP が大きく、位置精度が悪い可能性があります。")
                    elif hdop_value > 2.5:
                        warnings.append("HDOP がやや大きめです（中程度の精度）。")

                # 高度の簡易チェック
                try:
                    if altitude_raw not in (None, ""):
                        alt_val = float(altitude_raw)
                        if alt_val < -100 or alt_val > 10000:
                            warnings.append("高度が異常値の可能性があります（-100〜10000mの範囲外）。")
                except ValueError:
                    warnings.append(f"高度が数値として解釈できません: {altitude_raw}")

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

            except pynmea2.ParseError:
                # 解析エラーは無視して次の行へ
                continue

except serial.SerialException as e:
    print(f"エラー: {e}")
except KeyboardInterrupt:
    print("\nプログラムを終了します。")
finally:
    if "ser" in locals() and ser.is_open:
        ser.close()
    print("シリアルポートを閉じました。")