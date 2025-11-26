import serial
import time
import math
import threading
import datetime
import csv
import os
import pigpio
import sys
import cv2
from picamera2 import Picamera2

# --- 手動ライブラリのインポート ---
from library import bno055
from library import bmp180
from library.micropyGPS import MicropyGPS
from library import detect_corn as dc

class SimpleSonar:
    def __init__(self, pi_instance, trigger, echo):
        self.pi = pi_instance
        self.trig = trigger
        self.echo = echo
        self.high_tick = None
        self.distance = 999.0 # 初期値（遠い）

        # ピン設定
        self.pi.set_mode(self.trig, pigpio.OUTPUT)
        self.pi.set_mode(self.echo, pigpio.INPUT)
        self.pi.write(self.trig, 0)

        # コールバック設定 (立ち上がり・立ち下がりを監視)
        self.cb_rise = self.pi.callback(self.echo, pigpio.RISING_EDGE, self._cb)
        self.cb_fall = self.pi.callback(self.echo, pigpio.FALLING_EDGE, self._cb)

    def _cb(self, gpio, level, tick):
        if level == 1: # 立ち上がり
            self.high_tick = tick
        elif level == 0: # 立ち下がり
            if self.high_tick is not None:
                diff = pigpio.tickDiff(self.high_tick, tick)
                # 距離計算 (cm)
                dist = (diff / 1000000.0) * 34000 / 2
                if 2.0 < dist < 400.0:
                    self.distance = dist
                else:
                    self.distance = 999.0

    def trigger(self):
        """計測パルス発信"""
        self.pi.gpio_trigger(self.trig, 10, 1)

    def get_distance(self):
        return self.distance

# --- 定数設定 ---
# タイムアウト時間設定（秒）
TIMEOUT_PHASE_0 = 5 * 60   # 落下判定待ち最大
TIMEOUT_PHASE_1 = 30       # パラシュート分離最大
TIMEOUT_PHASE_2 = 2 * 60   # キャリブレーション最大
TIMEOUT_PHASE_3 = 5 * 60   # GPS誘導最大
TIMEOUT_PHASE_4 = 60       # コーン探索最大
TIMEOUT_PHASE_5 = 45       # 接近・スタック判定最大

DATA_SAMPLING_RATE = 0.01 

# --- Pin Number ---
# Motor Control Pins
PIN_ENA = 18
PIN_PHA = 8
PIN_ENB = 19
PIN_PHB = 9

# LED Pins
PIN_LED_1 = 5  # Status Indicator (Red recommended)
PIN_LED_2 = 6  # Activity Indicator (Green recommended)

PWM_FREQ = 20000
PWM_RANGE = 100

# Sonar Pins
PIN_TRIG = 23
PIN_ECHO = 24

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
sonar = None
obstacle_dist = 999.0

# ターゲット座標（適宜変更）
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
                time_phase1_start = time.time() # Phase1タイマースタート
            # ==========================================================
            # Phase 1: パラシュート分離
            # ==========================================================
            elif phase == 1:
                print("phase1 : removing parachute")

                pi.write(PIN_LED_1, 1) # Solid ON
                pi.write(PIN_LED_2, 0)
                
                direction = -400.0
                time.sleep(5)
                phase = 2
                time_phase2_start = time.time() # Phase2タイマースタート

            # ==========================================================
            # Phase 2: Calibration (キャリブレーション)
            # ==========================================================
            elif phase == 2:
                print("phase2 : BNO Calibration (Spinning)")
                pi.write(PIN_LED_1, 0)
                pi.write(PIN_LED_2, 1)
                
                calib_start_time = time.time()
                
                while True:
                    # ループ内LED点滅 (キャリブレーション中はチカチカさせる)
                    led_blink_timer += 1
                    toggle_led(PIN_LED_1, led_blink_timer, interval=3)
                    
                    # タイムアウト判定 (いつまでも補正が終わらない場合の強制脱出)
                    if time.time() - calib_start_time > TIMEOUT_PHASE_2:
                        print("Phase2 TIMEOUT: Force Phase 3 (Calibration Incomplete)")
                        break
                    
                    # キャリブレーション状態の確認
                    if bno is not None:
                        # getCalibrationStatus -> (sys, gyro, accel, mag) 各0-3
                        sys_st, gyro_st, accel_st, mag_st = bno.getCalibrationStatus()
                        
                        # ログ出力（デバッグ用）
                        if led_blink_timer % 10 == 0:
                            print(f"Calib Status: Sys={sys_st} Gyro={gyro_st} Acc={accel_st} Mag={mag_st}")
                        
                        # 判定基準: 地磁気(Mag)が 2以上になればOKとする
                        # (本来は3がベストだが、屋外/本番環境では3になりにくいことがある)
                        if mag_st >= 2:
                            print("Calibration OK! (Mag >= 2)")
                            break
                    else:
                        print("BNO None: Skip Calibration")
                        break
                        
                    time.sleep(0.1)
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

                # GPSが有効(lat != 0)なら誘導計算を行う
                if gps_detect == 1:
                    # 距離と方位を計算
                    # 現在地(lat, lng) -> ターゲット(TARGET_LAT, TARGET_LNG)
                    dist, azi = calc_distance_and_azimuth(lat, lng, TARGET_LAT, TARGET_LNG)
                    
                    # グローバル変数を更新
                    distance = dist
                    direction = azi       # これを更新すると moveMotor_thread が向きを変える
                    azimuth = azi         # ログ保存用
                    
                    # コンソール表示 (デバッグ用: 頻繁に出過ぎるならコメントアウト)
                    if led_blink_timer % 10 == 0:
                        print(f"GPS Nav: Dist={distance:.1f}m, TargetDir={direction:.1f}, MyHead={angle:.1f}")

                    # 接近判定 -> Phase 4へ
                    if distance < 5.0:
                        print(f"Close enough ({distance:.1f}m): Switching to Camera")
                        phase = 4
                        
                    # GPS補足時はLEDを早く点滅
                    toggle_led(PIN_LED_2, led_blink_timer, interval=2)
                    
                else:
                    # GPSロスト中
                    if led_blink_timer % 20 == 0:
                        print(f"GPS Lost: Keep going to {direction:.1f}...")
                    pass

                # 距離が近づいたらPhase4へ
                if distance < 5.0 and gps_detect == 1:
                    print("Close enough: Switching to Camera")
                    phase = 4
                    time_phase4_start = time.time() # Phase4タイマースタート

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
                        phase = 3
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
    global bno, bmp, detector, sonar
    
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

    # 3. Camera Setup ---
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
            pass
        
        # 2. ROIセット (Noneならデフォルト色モードになる)
        detector.set_roi_img(roi_img)
        
        # 3. テスト撮影（起動確認）
        detector.detect_cone()
        print("Camera: OK (Initialized)")

    # 4. Sonar Setup
    print("Sonar: Initializing...")
    try:
        # クラスを実体化 (グローバルの pi を渡す)
        sonar = SimpleSonar(pi, PIN_TRIG, PIN_ECHO)
        print("Sonar: OK")
    except Exception as e:
        print(f"Sonar Error: {e}")
        sonar = None
        
    except Exception as e:
        print(f"Camera: Critical Init Error {e}. Proceeding without Vision.")
        detector = None # 完全に死んでいる場合

    # 5. GPIO Setup
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

    # 6. Threads Start
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
                "ConeDir", "ConeProb", "ObstacleDist"
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

def getSonarData():
    """
    超音波センサーの値を安全に更新する
    エラーが起きても無視して、obstacle_dist を更新しない（初期値 or 前回の値のまま）
    """
    global obstacle_dist
    
    # センサー初期化に失敗していたら何もしない
    if sonar is None:
        return

    try:
        sonar.trigger() # パルス発信
        dist = sonar.get_distance() # 距離取得
        
        # 異常値フィルタリング（念のためここでもチェック）
        if 0 < dist < 500: 
            obstacle_dist = dist
            
    except Exception:
        pass # エラー時は無視（前回の値を維持）


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
# --- 追加するスレッド関数 ---

def setData_thread():
    """
    一定間隔でセンサー値を取得し、グローバル変数を更新＆ログ保存するスレッド
    """
    global acc, gyro, mag, lat, lng, alt, pres, distance, azimuth, angle, direction, fall, cone_direction, cone_probability, phase
    global obstacle_dist

    while True:
        # 1. 各種データの取得
        getBnoData() # 加速度・ジャイロ・磁気・落下判定(fall)を更新
        getBmpData() # 気圧・高度を更新
        getSonarData() # 超音波センサーの距離を更新

        # 2. 現在の方位(Heading)を個別に取得 (getBnoDataに含まれていないため)
        if bno is not None:
            try:
                # getEuler() -> [Heading, Roll, Pitch] (Heading: 0-360)
                euler = bno.getEuler()
                angle = euler[0] # グローバル変数の angle を更新
            except:
                pass
        if sonar is not None:
            sonar.trigger() # 音波発信
            obstacle_dist = sonar.get_distance() # 最新値取得

        # 3. ログファイルへの書き込み
        try:
            with open(fileName, "a", newline="") as f:
                writer = csv.writer(f)
                writer.writerow([
                    currentMilliTime(), phase,
                    f"{acc[0]:.2f}", f"{acc[1]:.2f}", f"{acc[2]:.2f}",
                    f"{gyro[0]:.2f}", f"{gyro[1]:.2f}", f"{gyro[2]:.2f}",
                    f"{mag[0]:.2f}", f"{mag[1]:.2f}", f"{mag[2]:.2f}",
                    f"{lat:.6f}", f"{lng:.6f}", f"{alt:.2f}", f"{pres:.2f}",
                    f"{distance:.2f}", f"{azimuth:.2f}", f"{angle:.2f}", 
                    f"{direction:.2f}", f"{fall:.2f}",
                    f"{cone_direction:.2f}", f"{cone_probability:.2f}",
                    f"{obstacle_dist:.2f}"
                ])
        except Exception as e:
            print(f"Log Error: {e}")

        # サンプリングレートに従って待機
        time.sleep(DATA_SAMPLING_RATE)


def moveMotor_thread():
    """
    現在の Phase とセンサー値に基づいてモーターを動かすスレッド
    """
    global direction, phase, cone_direction, angle
    global obstacle_dist
    
    # 基本スピード (0-100)
    BASE_SPEED = 60
    # 障害物回避距離
    AVOID_DIST = 30.0 # 30cm以下で回避行動

    while True:
        # ----------------------------------------
        # Phase 0: 落下中 & Phase 6: ゴール -> 停止
        # ----------------------------------------
        if phase == 0 or phase == 6:
            stop_motors()
            time.sleep(0.1)
            continue
        # 障害物回避 (超音波センサー)
        if phase not in [0, 1, 6] and obstacle_dist < AVOID_DIST:
            print(f"Obstacle Detected! {obstacle_dist:.1f}cm")
            
            # --- 回避動作 (バック＆ターン) ---
            stop_motors()
            time.sleep(0.2)
            
            # バック
            set_motor(PIN_ENA, PIN_PHA, 60, 0) # 0=Reverse
            set_motor(PIN_ENB, PIN_PHB, 60, 0)
            time.sleep(1.0)
            
            # 旋回 (右へ)
            set_motor(PIN_ENA, PIN_PHA, 60, 0) # 右後退
            set_motor(PIN_ENB, PIN_PHB, 60, 1) # 左前進
            time.sleep(0.5)
            
            stop_motors()
            time.sleep(0.2)
            continue # メインの制御をスキップしてループ先頭へ

        # ----------------------------------------
        # Phase 1: 回避行動 (direction = -400 の場合)
        # ----------------------------------------
        if phase == 1 and direction == -400.0:
            # 後退 (Reverse)
            # set_motor(ENピン, PHピン, 速度, 方向1/0)
            SEPARATION_SPEED = 100
            set_motor(PIN_ENA, PIN_PHA, SEPARATION_SPEED, 1) # 0=Reverse
            set_motor(PIN_ENB, PIN_PHB, SEPARATION_SPEED, 1)
            time.sleep(0.05)
            continue

        # ----------------------------------------
        # Phase 2: Calibration (キャリブレーション)
        # ----------------------------------------
        # 補足: その場で旋回して地磁気センサーを補正する
        if phase == 2:
            CALIB_SPEED = 50
            set_motor(PIN_ENA, PIN_PHA, CALIB_SPEED, 1) # 右: 前進
            set_motor(PIN_ENB, PIN_PHB, CALIB_SPEED, 0) # 左: 後退
            time.sleep(0.05)
            continue

        # ----------------------------------------
        # Phase 3: GPS誘導 (Heading制御)
        # ----------------------------------------
        if phase == 3:
            # 目標方位(direction) と 現在方位(angle) の差分を計算
            target_heading = direction + -5 # 種子島に合わせた補正値 (要調整)
            current_heading = angle
            
            diff = target_heading - current_heading
            # 差分を -180 〜 +180 に正規化
            if diff > 180:  diff -= 360
            if diff < -180: diff += 360
            
            # P制御 (比例制御)
            Kp = 0.5 # ゲイン (要調整)
            turn_val = diff * Kp
            
            # 出力制限 (急激な旋回を抑える)
            turn_val = max(-30, min(30, turn_val))
            
            # 左右のモーター速度を決定 (差動駆動)
            speed_L = BASE_SPEED + turn_val
            speed_R = BASE_SPEED - turn_val
            
            # 0-100の範囲に収める
            speed_L = max(0, min(100, speed_L))
            speed_R = max(0, min(100, speed_R))
            
            # 前進 (Forward=1)
            set_motor(PIN_ENA, PIN_PHA, speed_R, 1)
            set_motor(PIN_ENB, PIN_PHB, speed_L, 1)

        # ----------------------------------------
        # Phase 4: カメラ探索 (その場で旋回)
        # ----------------------------------------
        elif phase == 4:
            # 低速で右旋回してコーンを探す
            SEARCH_SPEED = 40
            set_motor(PIN_ENA, PIN_PHA, SEARCH_SPEED, 1) # 右: 前進
            set_motor(PIN_ENB, PIN_PHB, SEARCH_SPEED, 0) # 左: 後退
            
        # ----------------------------------------
        # Phase 5: カメラ接近 (画像認識による制御)
        # ----------------------------------------
        elif phase == 5:
            # cone_direction は 0.0(左端) ～ 1.0(右端)。 0.5が中央。
            center = 0.5
            err = cone_direction - center
            
            # P制御
            Kp_cam = 80 # ゲイン (要調整)
            turn_cam = err * Kp_cam
            
            speed_L = BASE_SPEED - turn_cam
            speed_R = BASE_SPEED + turn_cam
            
            speed_L = max(0, min(100, speed_L))
            speed_R = max(0, min(100, speed_R))
            
            set_motor(PIN_ENA, PIN_PHA, speed_R, 1)
            set_motor(PIN_ENB, PIN_PHB, speed_L, 1)

        time.sleep(0.05) # 制御周期

def currentMilliTime():
    return round(time.time() * 1000)

def set_motor(motor_pin_en, motor_pin_ph, speed, direction_val):
    pi.write(motor_pin_ph, direction_val)
    pi.set_PWM_dutycycle(motor_pin_en, int(speed))

def stop_motors():
    pi.set_PWM_dutycycle(PIN_ENA, 0)
    pi.set_PWM_dutycycle(PIN_ENB, 0)
    pi.write(PIN_PHA, 0)
    pi.write(PIN_PHB, 0)

def calc_distance_and_azimuth(lat1, lng1, lat2, lng2):
    """
    2点の緯度経度から距離(m)と方位角(度)を計算する
    lat1, lng1: 現在地
    lat2, lng2: ターゲット
    """
    # 地球の半径 (m)
    R = 6378137.0
    
    # ラジアンに変換
    rad_lat1 = math.radians(lat1)
    rad_lng1 = math.radians(lng1)
    rad_lat2 = math.radians(lat2)
    rad_lng2 = math.radians(lng2)
    
    # 距離の計算 (Haversine formula または 球面三角法)
    # ここでは簡易的な球面三角法を使用
    d_lng = rad_lng2 - rad_lng1
    
    # 距離計算
    sin_lat1 = math.sin(rad_lat1)
    cos_lat1 = math.cos(rad_lat1)
    sin_lat2 = math.sin(rad_lat2)
    cos_lat2 = math.cos(rad_lat2)
    cos_d_lng = math.cos(d_lng)
    
    # 中心角
    # arccosの引数が1を超えないようクリップ
    val = sin_lat1 * sin_lat2 + cos_lat1 * cos_lat2 * cos_d_lng
    val = max(-1.0, min(1.0, val))
    
    central_angle = math.acos(val)
    dist = R * central_angle
    
    # 方位角の計算 (Bearing)
    # 北を0度、時計回りの角度(0-360)を求める
    y = math.sin(d_lng) * cos_lat2
    x = cos_lat1 * sin_lat2 - sin_lat1 * cos_lat2 * cos_d_lng
    
    azi = math.degrees(math.atan2(y, x))
    
    # 0-360度に正規化
    if azi < 0:
        azi += 360.0
        
    return dist, azi