import serial
import time
import math
import threading
import datetime
import csv
import os
import sys
import cv2
from picamera2 import Picamera2

from gpiozero import LED, PWMOutputDevice, DigitalOutputDevice, DistanceSensor
from gpiozero.pins.lgpio import LGPIOFactory

# --- 手動ライブラリのインポート ---
from library import bno055
from library import bmp180
from library.micropyGPS import MicropyGPS
from library import detect_corn as dc

# --- グローバル状態管理クラス ---
class CanSatState:
    def __init__(self):
        self.lock = threading.Lock()
        self.acc = [0.0, 0.0, 0.0]
        self.gyro = [0.0, 0.0, 0.0]
        self.mag = [0.0, 0.0, 0.0]
        self.lat = 0.0
        self.lng = 0.0
        self.alt = 0.0
        self.pres = 0.0
        self.distance = 0.0
        self.azimuth = 0.0
        self.angle = 0.0
        self.direction = 0.0
        self.fall = 0.0
        self.cone_direction = 0.5
        self.cone_probability = 0.0
        self.obstacle_dist = 999.0
        self.phase = 0

    def update_imu(self, acc=None, gyro=None, mag=None, fall=None, angle=None):
        with self.lock:
            if acc is not None:
                self.acc = acc
            if gyro is not None:
                self.gyro = gyro
            if mag is not None:
                self.mag = mag
            if fall is not None:
                self.fall = fall
            if angle is not None:
                self.angle = angle

    def update_gps(self, lat=None, lng=None):
        with self.lock:
            if lat is not None:
                self.lat = lat
            if lng is not None:
                self.lng = lng

    def update_barometer(self, alt=None, pres=None):
        with self.lock:
            if alt is not None:
                self.alt = alt
            if pres is not None:
                self.pres = pres

    def update_navigation(self, distance=None, azimuth=None, direction=None, phase=None):
        with self.lock:
            if distance is not None:
                self.distance = distance
            if azimuth is not None:
                self.azimuth = azimuth
            if direction is not None:
                self.direction = direction
            if phase is not None:
                self.phase = phase

    def update_cone(self, cone_direction=None, cone_probability=None):
        with self.lock:
            if cone_direction is not None:
                self.cone_direction = cone_direction
            if cone_probability is not None:
                self.cone_probability = cone_probability

    def update_obstacle(self, obstacle_dist=None):
        with self.lock:
            if obstacle_dist is not None:
                self.obstacle_dist = obstacle_dist

    def snapshot(self):
        with self.lock:
            return {
                "acc": list(self.acc),
                "gyro": list(self.gyro),
                "mag": list(self.mag),
                "lat": self.lat,
                "lng": self.lng,
                "alt": self.alt,
                "pres": self.pres,
                "distance": self.distance,
                "azimuth": self.azimuth,
                "angle": self.angle,
                "direction": self.direction,
                "fall": self.fall,
                "cone_direction": self.cone_direction,
                "cone_probability": self.cone_probability,
                "obstacle_dist": self.obstacle_dist,
                "phase": self.phase,
            }

# インスタンス化して各スレッドに渡す
state = CanSatState()

# --- 定数設定 ---
# タイムアウト時間設定（秒）
TIMEOUT_PHASE_0 = 5 * 60   # 落下判定待ち最大
TIMEOUT_PHASE_1 = 30       # パラシュート分離最大
TIMEOUT_PHASE_2 = 2 * 60   # キャリブレーション最大
TIMEOUT_PHASE_3 = 5 * 60   # GPS誘導最大
TIMEOUT_PHASE_4 = 60       # コーン探索最大
TIMEOUT_PHASE_5 = 45       # 接近・スタック判定最大

DATA_SAMPLING_RATE = 0.06 # センサーデータ取得間隔（秒）

# --- Pin Number ---
# Motor Control Pins
PIN_ENA = 2
PIN_PHA = 13
PIN_ENB = 17
PIN_PHB = 19

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

# gpiozero オブジェクト格納用変数
led1 = None
led2 = None
motor_a_pwm = None
motor_a_dir = None
motor_b_pwm = None
motor_b_dir = None
sonar = None
pin_factory = None

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
    
    phase = 0 # スタートフェーズ（適宜変更）
    
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
                toggle_led(led1, led_blink_timer, interval=5)
                if led2:
                    led2.off()

                start = time.time()

                initial_alt = alt 
                print(f"Start Altitude: {initial_alt:.2f}m")

                while True:
                    # 落下判定ループ内でも点滅させる
                    led_blink_timer += 1
                    toggle_led(led1, led_blink_timer, interval=5)

                    # 1. 加速度判定
                    is_impact = (fall > 30.0) # 30 m/s^2 ≒ 3G (閾値は要検討)

                    # 2. 高度判定
                    altitude_diff = initial_alt - alt
                    is_drop = (altitude_diff > 60.0) # 60m以上の降下で落下と判定（閾値は要検討）

                    # 判定ロジック: どちらか、または両方を満たしたらパラシュート開傘とみなす
                    if is_drop:
                        print(f"Detected Drop: {altitude_diff:.2f}m")
                        break
                    
                    if is_impact:
                        print(f"Detected Impact: {fall:.2f}m/s^2")
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

                if led1:
                    led1.on() # Solid ON
                if led2:
                    led2.off()
                
                direction = -400.0
                time.sleep(5)
                phase = 2
                time_phase2_start = time.time() # Phase2タイマースタート

            # ==========================================================
            # Phase 2: Calibration (キャリブレーション)
            # ==========================================================
            elif phase == 2:
                print("phase2 : BNO Calibration (Spinning)")
                if led1:
                    led1.off()
                if led2:
                    led2.on()
                
                calib_start_time = time.time()
                
                while True:
                    # ループ内LED点滅 (キャリブレーション中はチカチカさせる)
                    led_blink_timer += 1
                    toggle_led(led1, led_blink_timer, interval=3)
                    
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
                if led1:
                    led1.off()
                toggle_led(led2, led_blink_timer, interval=10) # 1秒間隔

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
                    toggle_led(led2, led_blink_timer, interval=2)
                    
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
                if led1:
                    led1.off()
                if led2:
                    led2.on() # Solid ON
                
                # カメラが死んでいても cone_detect はエラーを吐かずに戻ってくる
                # cone_detect() ←スレッドで実行するためコメントアウト
                
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
                        if led1:
                            led1.on()
                        if led2:
                            led2.off()
                    else:
                        if led1:
                            led1.off()
                        if led2:
                            led2.on()

                    # cone_detect() ← スレッドで実行するためコメントアウト
                    
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

                    # スタック/カメラ故障時のタイムアウト
                    if time.time() - time_camera_start >= TIMEOUT_PHASE_5:
                        print("Phase5 TIMEOUT: Giving up, forcing Goal")
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
                if led1:
                    led1.on()
                if led2:
                    led2.on()
                stop_motors()
                sys.exit()

            time.sleep(0.1)

    except KeyboardInterrupt:
        print("\nKeyboardInterrupt")
        stop_motors()
        sys.exit()

# --- LED Helper Functions ---

def toggle_led(led, timer, interval):
    """
    指定したインターバル(ループ回数)ごとにLEDを反転させる
    """
    if led is None:
        return
    if (timer // interval) % 2 == 0:
        led.on()
    else:
        led.off()

def signal_led(times):
    """
    指定回数だけ両方のLEDを点滅させる（通知用）
    """
    for _ in range(times):
        if led1:
            led1.on()
        if led2:
            led2.on()
        time.sleep(0.2)
        if led1:
            led1.off()
        if led2:
            led2.off()
        time.sleep(0.2)

# --- Robust Setup Function ---
def Setup():
    global bno, bmp, detector, sonar, pin_factory
    global led1, led2, motor_a_pwm, motor_a_dir, motor_b_pwm, motor_b_dir
    
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
    except Exception as e:
        print(f"BNO055: Critical Error {e}. Proceeding.")
        bno = None

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

    # 3. Camera Setup
    print("Camera: Initializing...")
    try:
        detector = dc.detector()

        roi_path_1 = "./log/captured_roi_img.png"
        roi_path_2 = "./log/captured.png"
        roi_img = None
        if os.path.exists(roi_path_1):
            print(f"Loading ROI from {roi_path_1}")
            roi_img = cv2.imread(roi_path_1)
        elif os.path.exists(roi_path_2):
            print(f"Loading ROI from {roi_path_2}")
            roi_img = cv2.imread(roi_path_2)
        else:
            print("WARNING: No ROI image found. Switching to DEFAULT ORANGE detection.")

        detector.set_roi_img(roi_img)
        detector.detect_cone()
        print("Camera: OK (Initialized)")
    except Exception as e:
        print(f"Camera: Critical Init Error {e}. Proceeding without Vision.")
        detector = None

    # 4. GPIOZero Setup (LED, Motor, Sonar)
    print("GPIOZero: Initializing devices...")
    try:
        pin_factory = LGPIOFactory()
        led1 = LED(PIN_LED_1, pin_factory=pin_factory)
        led2 = LED(PIN_LED_2, pin_factory=pin_factory)
        motor_a_pwm = PWMOutputDevice(PIN_ENA, pin_factory=pin_factory, frequency=PWM_FREQ, initial_value=0)
        motor_a_dir = DigitalOutputDevice(PIN_PHA, pin_factory=pin_factory, initial_value=False)
        motor_b_pwm = PWMOutputDevice(PIN_ENB, pin_factory=pin_factory, frequency=PWM_FREQ, initial_value=0)
        motor_b_dir = DigitalOutputDevice(PIN_PHB, pin_factory=pin_factory, initial_value=False)
        sonar = DistanceSensor(echo=PIN_ECHO, trigger=PIN_TRIG, max_distance=4.0, pin_factory=pin_factory)
        stop_motors()
        print("GPIOZero: OK")
    except Exception as e:
        print(f"GPIOZero Setup Error {e}. Motors/LED/Sonar might not work.")
        led1 = led2 = motor_a_pwm = motor_a_dir = motor_b_pwm = motor_b_dir = sonar = None

    # 5. Threads Start
    try:
        threading.Thread(target=moveMotor_thread, daemon=True).start()
        threading.Thread(target=setData_thread, daemon=True).start()
        threading.Thread(target=GPS_thread, daemon=True).start()
        threading.Thread(target=camera_thread, daemon=True).start()

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
    except Exception:
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
        dist_m = sonar.distance  # DistanceSensorはメートル単位
        dist_cm = dist_m * 100.0
        if 0 < dist_cm < 500:
            obstacle_dist = dist_cm
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

                # デバッグ用
                # 実際の戻り値を表示して、[0]でアクセスして良いか確認する
                # 本番ではコメントアウトする
                print(f"DEBUG GPS LAT TYPE: {type(gps.latitude)}, VALUE: {gps.latitude}")
                # ----------------------

                lat = gps.latitude[0]
                lng = gps.longitude[0]
                
                if lat != 0.0:
                    gps_detect = 1
                else:
                    gps_detect = 0
        except:
            pass # 読み取りエラー無視


def camera_thread():
    """
    カメラ処理を独立して行うスレッド
    Phase 4, 5 のときだけ detect_cone を実行してグローバル変数を更新する
    """
    global phase
    
    while True:
        # カメラが必要なフェーズのみ処理を実行
        if phase in [4, 5]:
            # 既存の cone_detect() 関数を呼べばOK
            # 内部で detector.detect_cone() が呼ばれ、重い処理が走る
            cone_detect()
            
            # 処理頻度の調整 (例: 最大20fps程度に制限してCPUを休ませる)
            time.sleep(0.05)
        else:
            # カメラ不要なフェーズはスリープ長めにしてCPU負荷を下げる
            time.sleep(0.5)

# --- Helper Functions ---
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
            try:
                obstacle_dist = sonar.distance * 100.0
            except Exception:
                pass

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

# --- Motor Control Functions ---
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
        if phase not in [0, 1, 5, 6] and obstacle_dist < AVOID_DIST:
            print(f"Obstacle Detected! {obstacle_dist:.1f}cm")
            
            # --- 回避動作 (バック＆ターン) ---
            stop_motors()
            time.sleep(0.2)
            
            # バック
            set_motor(motor_a_pwm, motor_a_dir, 60, 0) # 0=Reverse
            set_motor(motor_b_pwm, motor_b_dir, 60, 0)
            time.sleep(1.0)
            
            # 旋回 (右へ)
            set_motor(motor_a_pwm, motor_a_dir, 60, 0) # 右後退
            set_motor(motor_b_pwm, motor_b_dir, 60, 1) # 左前進
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
            set_motor(motor_a_pwm, motor_a_dir, SEPARATION_SPEED, 1) # 0=Reverse
            set_motor(motor_b_pwm, motor_b_dir, SEPARATION_SPEED, 1)
            time.sleep(0.05)
            continue

        # ----------------------------------------
        # Phase 2: Calibration (キャリブレーション)
        # ----------------------------------------
        # 補足: その場で旋回して地磁気センサーを補正する
        if phase == 2:
            CALIB_SPEED = 50
            set_motor(motor_a_pwm, motor_a_dir, CALIB_SPEED, 1) # 右: 前進
            set_motor(motor_b_pwm, motor_b_dir, CALIB_SPEED, 0) # 左: 後退
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
            set_motor(motor_a_pwm, motor_a_dir, speed_R, 1)
            set_motor(motor_b_pwm, motor_b_dir, speed_L, 1)

        # ----------------------------------------
        # Phase 4: カメラ探索 (その場で旋回)
        # ----------------------------------------
        elif phase == 4:
            # 低速で右旋回してコーンを探す
            SEARCH_SPEED = 40
            set_motor(motor_a_pwm, motor_a_dir, SEARCH_SPEED, 1) # 右: 前進
            set_motor(motor_b_pwm, motor_b_dir, SEARCH_SPEED, 0) # 左: 後退
            
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
            
            speed_L = BASE_SPEED + turn_cam
            speed_R = BASE_SPEED - turn_cam
            
            speed_L = max(0, min(100, speed_L))
            speed_R = max(0, min(100, speed_R))
            
            set_motor(motor_a_pwm, motor_a_dir, speed_R, 1)
            set_motor(motor_b_pwm, motor_b_dir, speed_L, 1)

        time.sleep(0.05) # 制御周期

def currentMilliTime():
    return round(time.time() * 1000)

def set_motor(motor_pwm, motor_dir, speed, forward):
    """
    Drive a motor with gpiozero devices. speed: 0-100, forward: bool
    """
    if motor_pwm is None or motor_dir is None:
        return
    motor_dir.value = 1 if forward else 0
    motor_pwm.value = max(0.0, min(1.0, speed / 100.0))

def stop_motors():
    if motor_a_pwm:
        motor_a_pwm.value = 0
    if motor_b_pwm:
        motor_b_pwm.value = 0

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
