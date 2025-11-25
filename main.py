import serial
import time
import math
import threading
import datetime
import csv
import os
import pigpio
import sys

# --- ライブラリのインポート ---
from library import bno055
from library import bmp180
from library.micropyGPS import MicropyGPS
from library import detect_corn as dc
from picamera2 import Picamera2

# --- 定数設定 ---
# タイムアウト時間設定（秒）
TIMEOUT_PHASE_0 = 5 * 60   # 落下判定待ち最大
TIMEOUT_PHASE_3 = 3 * 60   # GPS誘導最大
TIMEOUT_PHASE_4 = 20       # カメラ探索最大
TIMEOUT_PHASE_5 = 45       # 接近・スタック判定最大

DATA_SAMPLING_RATE = 0.01 

# --- Pin Number ---
# Motor Control Pins
PIN_ENA = 18
PIN_PHA = 8
PIN_ENB = 19
PIN_PHB = 9

# LEDインジケーター
PIN_LED_1 = 5  # Status Indicator (Red recommended)
PIN_LED_2 = 6  # Activity Indicator (Green recommended)

PWM_FREQ = 20000
PWM_RANGE = 100

# --- グローバル変数 (初期値: 安全なデフォルト値) ---
acc = [0.0, 0.0, 0.0]
gyro = [0.0, 0.0, 0.0]
mag = [0.0, 0.0, 0.0]
lat = 0.0
lng = 0.0
alt = 0.0
pres = 0.0
distance = 9999.0 # 初期値は遠くにしておく
angle = 0.0
azimuth = 0.0
direction = 0.0
phase = 0
gps_detect = 0
cone_direction = 0.5
cone_probability = 0
fall = 0.0
upside_down_Flag = 0

# ターゲット座標
TARGET_LAT = 38.26052
TARGET_LNG = 140.8544151

# インスタンス格納用（初期化失敗時はNoneのままにする）
bno = None
bmp = None
detector = None

# pigpio初期化
pi = pigpio.pi()
if not pi.connected:
    print("Error: pigpio daemon not running. Run 'sudo pigpiod'.")
    sys.exit()

# ログファイル
nowTime = datetime.datetime.now()
os.makedirs("./log", exist_ok=True)
fileName = "./log/robust_log_" + nowTime.strftime("%Y-%m%d-%H%M%S") + ".csv"


def main():
    global phase, start
    
    # Flags
    searching_Flag = False
    count_cone_lost = 0
    
    # Timers
    time_phase3_start = 0
    time_start_searching_cone = 0
    time_camera_start = 0

    # LED点滅用カウンター
    led_blink_timer = 0

    # 全センサーの初期化（失敗しても進む）
    Setup()

    # スタート時のLED合図 (3回点滅)
    signal_led(3)
    
    phase = 3 # スタートフェーズ（適宜0に変更してください）
    
    try:
        while True:
            # カウンター更新 (約0.1秒ごとに加算)
            led_blink_timer += 1

            # ==========================================================
            # Phase 0: Falling (落下検知)
            # ==========================================================
            if phase == 0:
                print("phase0 : falling")
                # LED制御: 0.5秒間隔で点滅 (5 * 0.1s)
                toggle_led(PIN_LED_1, led_blink_timer, interval=5)
                pi.write(PIN_LED_2, 0)

                start = time.time()
                while True:
                    # 落下判定ループ内でも点滅させる
                    led_blink_timer += 1
                    toggle_led(PIN_LED_1, led_blink_timer, interval=5)

                    # BNOが死んでいても time.time() で強制脱出
                    if fall > 25:
                        print("Detected Fall (Para released)")
                        time.sleep(10)
                        break
                    
                    if time.time() - start > TIMEOUT_PHASE_0:
                        print("Phase0 TIMEOUT: Force proceed (Sensor failure?)")
                        break
                    time.sleep(0.1)
                phase = 1

            # ==========================================================
            # Phase 1: Avoidance (回避行動)
            # ==========================================================
            elif phase == 1:
                print("phase1 : avoidance")

                pi.write(PIN_LED_1, 1) # Solid ON
                pi.write(PIN_LED_2, 0)
                
                direction = -400.0
                time.sleep(5)
                phase = 3
                time_phase3_start = time.time() # Phase3タイマースタート

            # ==========================================================
            # Phase 3: GPS Navigation (GPS誘導)
            # ==========================================================
            elif phase == 3:
                pi.write(PIN_LED_1, 0)
                toggle_led(PIN_LED_2, led_blink_timer, interval=10) # 1秒間隔

                # GPSが死んでいる、または到達できない場合のタイムアウト処理
                if time.time() - time_phase3_start > TIMEOUT_PHASE_3:
                    print("Phase3 TIMEOUT: Give up GPS, switching to Camera")
                    phase = 4
                    continue

                # 簡易誘導ロジック
                # GPSが取れていない(lat=0)場合は直進させる、などの安全策
                if gps_detect == 1:
                    # 本来の誘導ロジック
                    # direction = ...
                    direction = 360.0 # 仮
                    # GPS補足時はLED 2を早く点滅させるなどの変化も可能
                    toggle_led(PIN_LED_2, led_blink_timer, interval=2)
                else:
                    # GPS取れないならとりあえず直進? 停止?
                    print("Waiting for GPS fix...")
                    direction = 360.0 # 仮:停止待機

                # 距離が近づいたらPhase4へ (GPS生きてる場合)
                if distance < 5.0 and gps_detect == 1:
                    print("Close enough: Switching to Camera")
                    phase = 4

            # ==========================================================
            # Phase 4: Camera Searching (探索)
            # LED動作: LED 2 点灯 (カメラ注視中)
            # ==========================================================
            elif phase == 4:
                print("phase4 : camera searching")
                pi.write(PIN_LED_1, 0)
                pi.write(PIN_LED_2, 1) # Solid ON
                
                # カメラが死んでいても cone_detect はエラーを吐かずに戻ってくる
                cone_detect()
                
                if not searching_Flag:
                    searching_Flag = True
                    time_start_searching_cone = time.time()
                else:
                    # 首振りしても見つからない、またはカメラ故障時のタイムアウト
                    if time.time() - time_start_searching_cone >= TIMEOUT_PHASE_4:
                        print("Camera TIMEOUT: Cone not found or Camera dead")
                        searching_Flag = False
                        # ここでPhase3に戻るか、Phase5に強行するかは戦略次第
                        # 今回はGPSに戻る（そしてGPSもタイムアウトしたらまた来る）
                        phase = 3 
                        time_phase3_start = time.time() # タイマーリセット
                
                # 見つかったら
                if cone_probability > 0.1:
                    phase = 5

            # ==========================================================
            # Phase 5: Approach (接近)
            # LED動作: 両方のLEDが交互に点滅 (緊急/接近モード)
            # ==========================================================
            elif phase == 5:
                print("phase5 : approaching")
                time_camera_start = time.time()
                count_cone_lost = 0
                
                while True:
                    # ループ内LED制御
                    led_blink_timer += 1
                    # 交互点滅 (0.2秒間隔)
                    if (led_blink_timer // 2) % 2 == 0:
                        pi.write(PIN_LED_1, 1)
                        pi.write(PIN_LED_2, 0)
                    else:
                        pi.write(PIN_LED_1, 0)
                        pi.write(PIN_LED_2, 1)

                    cone_detect()
                    
                    # カメラ故障時は detector.is_detected は常に False になる想定
                    is_det = False
                    is_reach = False
                    if detector is not None:
                        is_det = detector.is_detected
                        is_reach = detector.is_reached
                    
                    if not is_det:
                        count_cone_lost += 1
                    else:
                        count_cone_lost = 0
                    
                    # 見失いリトライ
                    if count_cone_lost >= 10:
                        phase = 4
                        break

                    # スタック/カメラ故障時の強制ゴール判定
                    if time.time() - time_camera_start >= TIMEOUT_PHASE_5:
                        print("Phase5 TIMEOUT: Force Goal (Assume reached or Stuck)")
                        phase = 6
                        break
                        
                    if is_reach:
                        print("Reached Cone! (Visual confirmation)")
                        phase = 6
                        break
                    
                    time.sleep(0.1)

            # ==========================================================
            # Phase 6: Goal
            # LED動作: 両方点灯 (完了)
            # ==========================================================
            elif phase == 6:
                print("phase6 : Goal!!")
                pi.write(PIN_LED_1, 1)
                pi.write(PIN_LED_2, 1)
                stop_motors()
                pi.stop()
                sys.exit()

            time.sleep(0.1)

    except KeyboardInterrupt:
        print("\nKeyboardInterrupt")
        stop_motors()
        pi.stop()
        sys.exit()

# --- LED Helper Functions ---

def toggle_led(pin, timer, interval):
    """
    指定したインターバル(ループ回数)ごとにLEDを反転させる
    """
    if (timer // interval) % 2 == 0:
        pi.write(pin, 1)
    else:
        pi.write(pin, 0)

def signal_led(times):
    """
    指定回数だけ両方のLEDを点滅させる（通知用）
    """
    for _ in range(times):
        pi.write(PIN_LED_1, 1)
        pi.write(PIN_LED_2, 1)
        time.sleep(0.2)
        pi.write(PIN_LED_1, 0)
        pi.write(PIN_LED_2, 0)
        time.sleep(0.2)


def Setup():
    global bno, bmp, detector
    
    print("--- Robust Setup Start ---")
    
    # 1. BNO055 Setup (Retry & Ignore)
    try:
        bno = bno055.BNO055()
        for i in range(3):
            if bno.setUp():
                print("BNO055: OK")
                break
            print(f"BNO055: Retry {i+1}...")
            time.sleep(0.5)
        else:
            print("WARNING: BNO055 Init Failed. Proceeding anyway.")
            # bnoオブジェクトは残すが、中身が正常でない可能性あり
    except Exception as e:
        print(f"BNO055: Critical Error {e}. Proceeding.")
        bno = None # Noneにしておき、後段でチェックする

    # 2. BMP180 Setup (Ignore on fail)
    try:
        bmp = bmp180.BMP180(oss=3)
        if bmp.setUp():
            print("BMP180: OK")
        else:
            print("WARNING: BMP180 Init Failed. Proceeding.")
    except Exception as e:
        print(f"BMP180: Critical Error {e}. Proceeding.")
        bmp = None

    # --- Camera Setup ---
    print("Camera: Initializing...")
    try:
        detector = dc.detector() # インスタンス作成
        
        # 1. 画像ファイルの読み込み (ロバスト化)
        # 優先順位: capture_roi_image.pyの保存名 -> ユーザー指定名 -> None
        roi_path_1 = "./log/captured_roi_img.png" # capture_roi_img.py の保存名
        roi_path_2 = "./log/captured.png"         # main.py の元の記述
        
        roi_img = None
        
        if os.path.exists(roi_path_1):
            print(f"Loading ROI from {roi_path_1}")
            roi_img = cv2.imread(roi_path_1)
        elif os.path.exists(roi_path_2):
            print(f"Loading ROI from {roi_path_2}")
            roi_img = cv2.imread(roi_path_2)
        else:
            print("WARNING: No ROI image found. Switching to DEFAULT ORANGE detection.")
        
        if roi_img is not None:
             # BGR -> RGB変換は detect_corn 内で BGR2HSV するので、
             # cv2.imread (BGR) のままで渡すのが正解です。
             # 元のコードにあった cv2.cvtColor(..., RGB) は detect_cornの実装によってはバグの元ですが、
             # 今回提示した新detect_corn.pyは BGR入力を想定しているのでそのまま渡します。
             pass

        # 2. ROIセット (Noneならデフォルト色モードになる)
        detector.set_roi_img(roi_img)
        
        # 3. テスト撮影（起動確認）
        detector.detect_cone()
        print("Camera: OK (Initialized)")
        
    except Exception as e:
        print(f"Camera: Critical Init Error {e}. Proceeding without Vision.")
        detector = None # 完全に死んでいる場合

    # 4. GPIO Setup
    try:
        pi.set_mode(PIN_ENA, pigpio.OUTPUT)
        pi.set_mode(PIN_ENB, pigpio.OUTPUT)
        pi.set_mode(PIN_PHA, pigpio.OUTPUT)
        pi.set_mode(PIN_PHB, pigpio.OUTPUT)

        pi.set_PWM_range(PIN_ENA, PWM_RANGE)
        pi.set_PWM_range(PIN_ENB, PWM_RANGE)
        pi.set_PWM_frequency(PIN_ENA, PWM_FREQ)
        pi.set_PWM_frequency(PIN_ENB, PWM_FREQ)
        stop_motors()
    except Exception as e:
        print(f"GPIO Setup Error {e}. Motors might not work.")

    # 5. Threads Start
    # スレッド起動もtryで囲む（念のため）
    try:
        threading.Thread(target=moveMotor_thread, daemon=True).start()
        threading.Thread(target=setData_thread, daemon=True).start()
        threading.Thread(target=GPS_thread, daemon=True).start()
    except Exception as e:
        print(f"Thread Start Error {e}.")

    # Log File Init
    try:
        with open(fileName, "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow([
                "MilliTime", "Phase", 
                "AccX", "AccY", "AccZ", "GyroX", "GyroY", "GyroZ", "MagX", "MagY", "MagZ",
                "LAT", "LNG", "ALT", "Pres",
                "Distance", "Azimuth", "Angle", "Direction", "Fall", 
                "ConeDir", "ConeProb"
            ])
    except:
        print("Log File Init Failed. No logging.")

    print("--- Setup Finished (Ready to Die Trying) ---")


# --- 安全なデータ取得関数群 ---

def getBnoData():
    global acc, gyro, mag, fall
    if bno is None: return # センサーがないなら何もしない(初期値0のまま)
    
    try:
        new_acc = bno.getAcc()
        new_gyro = bno.getGyro()
        new_mag = bno.getMag()
        
        # Noneチェック
        if new_acc: acc = new_acc
        if new_gyro: gyro = new_gyro
        if new_mag: mag = new_mag
        
        fall = math.sqrt(acc[0]**2 + acc[1]**2 + acc[2]**2)
    except:
        pass # エラーは無視

def getBmpData():
    global alt, pres
    if bmp is None: return
    
    try:
        alt = bmp.getAltitude()
        pres = bmp.getPressure()
    except:
        pass

def cone_detect():
    global cone_direction, cone_probability
    # カメラが死んでいたら(detector is None)、何もしない
    if detector is None:
        cone_direction = 0.5 # 中央
        cone_probability = 0.0 # 発見できず
        return

    try:
        detector.detect_cone()
        # 確率更新
        cone_probability = detector.probability if detector.probability else 0.0
        # 方向更新 (detect_corn.pyの実装に依存、エラー時は中央へ)
        if detector.cone_direction is not None:
            cone_direction = 1.0 - detector.cone_direction
        else:
            cone_direction = 0.5
    except:
        # カメラ処理中のOpenCVエラーなどもここで握りつぶす
        cone_direction = 0.5
        cone_probability = 0.0


def GPS_thread():
    global lat, lng, gps_detect
    
    # シリアル接続試行
    s = None
    try:
        s = serial.Serial("/dev/serial0", 115200, timeout=1)
    except:
        print("GPS Serial Open Failed. GPS is DEAD.")
        # returnせず、ループに入って（何もしないけど）スレッドを維持する手もあるが
        # ここでは座標0のまま静かに終了させないでおく
        pass

    gps = MicropyGPS(9, "dd")

    while True:
        if s is None:
            time.sleep(1) # シリアルが死んでるなら寝て待つ（再接続ロジック入れてもいいが今回は省略）
            continue
            
        try:
            line = s.readline().decode("utf-8", errors="ignore")
            if len(line) > 0 and line[0] == '$':
                for x in line:
                    gps.update(x)
                lat = gps.latitude[0]
                lng = gps.longitude[0]
                
                if lat != 0.0:
                    gps_detect = 1
                else:
                    gps_detect = 0
        except:
            pass # 読み取りエラー無視

# --- Helper Functions ---

def currentMilliTime():
    return round(time.time() * 1000)

def set_motor(motor_pin_en, motor_pin_ph, speed, direction_val):
    pi.write(motor_pin_ph, direction_val)
    pi.set_PWM_dutycycle(motor_pin_en, int(speed))

def stop_motors():
    pi.set_PWM_dutycycle(PIN_ENA, 0)
    pi.set_PWM_dutycycle(PIN_ENB, 0)
    pi.write(PIN_PHA, 0)
    pi.write