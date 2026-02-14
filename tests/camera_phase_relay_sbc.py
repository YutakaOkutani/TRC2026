import argparse, base64, json, math, os, socket, struct, threading, time
import cv2, pynmea2, serial
from gpiozero import DigitalOutputDevice, LED, PWMOutputDevice
from gpiozero.pins.lgpio import LGPIOFactory
from library import bno055
from library import detect_corn as dc

LOG_DIR = './log'
ROI_PATH_1 = os.path.join(LOG_DIR, 'captured_roi_img.png')
ROI_PATH_2 = os.path.join(LOG_DIR, 'captured.png')

CONE_PROBABILITY_THRESHOLD = 0.1
CONE_LOST_COUNT_LIMIT = 10
CONE_CENTER_POSITION = 0.5
TIMEOUT_PHASE_4 = 60
TIMEOUT_PHASE_5 = 45
CAMERA_ACTIVE_SLEEP = 0.05
CAMERA_IDLE_SLEEP = 0.5
CAMERA_REINIT_INTERVAL = 5.0
CAMERA_FAIL_LIMIT = 5
CAMERA_DEAD_TIMEOUT = 30.0
CAMERA_PHASE4_MAX_ATTEMPTS = 3
CAMERA_PHASE5_MAX_ATTEMPTS = 3
SEARCH_ROTATION_SPEED = 40
APPROACH_TURN_GAIN = 80
BASE_SPEED = 60
MOTOR_LOOP_INTERVAL = 0.05
MOTOR_RAMP_TIME = 0.6
MOTOR_RAMP_STEP = 0.05
MOTOR_DIR_INVERT_1 = True
MOTOR_DIR_INVERT_2 = True
LED_INTERVAL_PHASE5 = 2
PIN_EN1 = 12
PIN_PH1 = 13
PIN_EN2 = 19
PIN_PH2 = 17
PWM_FREQ = 1000
PIN_LED_RED = 5
PIN_LED_GREEN = 6
GPS_SERIAL_PORT = '/dev/serial0'
GPS_SERIAL_PORT_CANDIDATES = ['/dev/serial0', '/dev/ttyAMA0', '/dev/ttyS0']
GPS_BAUDRATE = 9600
GPS_BAUDRATE_CANDIDATES = [9600]
GPS_SERIAL_TIMEOUT = 1
GPS_BUFFER_CLEAR_THRESHOLD = 2048
GPS_BUFFER_CLEAR_INTERVAL = 5.0
GPS_MIN_FIX_QUAL = 1
GPS_MIN_SATELLITES = 4
GPS_MAX_HDOP = 5.0
GPS_MAX_SPEED_MPS = 10.0
GPS_STABLE_FIX_COUNT = 3
GPS_FIX_LOSS_TIMEOUT = 8.0
GPS_HEADING_MIN_DIST = 0.5
BNO_SETUP_RETRY_COUNT = 3
BNO_SETUP_RETRY_INTERVAL = 0.5


def calc_distance_and_azimuth(lat1, lng1, lat2, lng2):
    r = 6378137.0
    a1, o1, a2, o2 = map(math.radians, [lat1, lng1, lat2, lng2])
    d = o2 - o1
    v = math.sin(a1) * math.sin(a2) + math.cos(a1) * math.cos(a2) * math.cos(d)
    v = max(-1.0, min(1.0, v))
    dist = r * math.acos(v)
    y = math.sin(d) * math.cos(a2)
    x = math.cos(a1) * math.sin(a2) - math.sin(a1) * math.cos(a2) * math.cos(d)
    azi = math.degrees(math.atan2(y, x))
    return dist, azi + 360.0 if azi < 0 else azi


class State:
    def __init__(self, phase=4):
        self.lock = threading.Lock()
        self.phase = phase
        self.cone_probability = 0.0
        self.cone_direction = CONE_CENTER_POSITION
        self.cone_is_reached = False
        self.camera_debug = {'detected': False, 'centroid_px': None, 'bbox_px': None, 'goal_sign': False, 'message': ''}
        self.acc = [0.0, 0.0, 0.0]
        self.gyro = [0.0, 0.0, 0.0]
        self.mag = [0.0, 0.0, 0.0]
        self.fall = 0.0
        self.angle = 0.0
        self.angle_valid = False
        self.bno_stale_sec = 0.0
        self.lat = 0.0
        self.lng = 0.0
        self.gps_detect = 0
        self.gps_heading = None
        self.gps_heading_valid = False
        self.num_sats = None
        self.hdop = None
        self.gps_qual = None
        self.frame_b64 = None
        self.frame_seq = 0

    def snapshot(self):
        with self.lock:
            return dict(self.__dict__)


class Relay:
    def __init__(self, args):
        self.args = args
        self.state = State(args.start_phase)
        self.stop = threading.Event()
        self.detector = None
        self.roi_img = None
        self.camera_fail_count = 0
        self.camera_last_reinit = 0.0
        self.camera_dead_since = None
        self.bno = None
        self.bno_last_valid_time = 0.0
        self.devices = {'led_red': None, 'led_green': None, 'motor_1_pwm': None, 'motor_1_dir': None, 'motor_2_pwm': None, 'motor_2_dir': None}
        self.motor_state = {}
        self.led_blink_timer = 0
        self.searching_flag = False
        self.count_cone_lost = 0
        self.time_start_searching_cone = 0
        self.time_camera_start = 0
        self.camera_phase4_attempts = 0
        self.camera_phase5_attempts = 0
        self.camera_phase4_start = None
        self.camera_phase5_start = None

    def setup(self):
        try:
            self.detector = dc.detector(); roi = None
            if os.path.exists(ROI_PATH_1): roi = cv2.imread(ROI_PATH_1)
            elif os.path.exists(ROI_PATH_2): roi = cv2.imread(ROI_PATH_2)
            self.roi_img = roi
            self.detector.set_roi_img(roi)
            self.detector.detect_cone()
        except Exception:
            self.detector = None
        try:
            b = bno055.BNO055()
            for _ in range(BNO_SETUP_RETRY_COUNT):
                if b.setUp(): self.bno = b; break
                time.sleep(BNO_SETUP_RETRY_INTERVAL)
        except Exception:
            self.bno = None
        try:
            f = LGPIOFactory()
            self.devices['led_red'] = LED(PIN_LED_RED, pin_factory=f)
            self.devices['led_green'] = LED(PIN_LED_GREEN, pin_factory=f)
            self.devices['motor_1_pwm'] = PWMOutputDevice(PIN_EN1, pin_factory=f, frequency=PWM_FREQ, initial_value=0)
            self.devices['motor_1_dir'] = DigitalOutputDevice(PIN_PH1, pin_factory=f, initial_value=False)
            self.devices['motor_2_pwm'] = PWMOutputDevice(PIN_EN2, pin_factory=f, frequency=PWM_FREQ, initial_value=0)
            self.devices['motor_2_dir'] = DigitalOutputDevice(PIN_PH2, pin_factory=f, initial_value=False)
            for k in ('motor_1_pwm', 'motor_2_pwm'):
                self.motor_state[self.devices[k]] = {'speed': 0.0, 'direction': True}
            self.stop_motors()
        except Exception:
            pass

    def _try_reinit_camera(self):
        now = time.time()
        if now - self.camera_last_reinit < CAMERA_REINIT_INTERVAL: return
        self.camera_last_reinit = now
        try:
            d = dc.detector(); d.set_roi_img(self.roi_img); d.detect_cone(); self.detector = d
            self.camera_fail_count = 0; self.camera_dead_since = None
        except Exception:
            pass

    def _extract_box(self):
        d = self.detector
        if d is None or d.binarized_img is None: return None, None
        n, _, st, ct = cv2.connectedComponentsWithStats(d.binarized_img.astype('uint8'))
        if n <= 1: return None, None
        st, ct = st[1:], ct[1:]
        occ = st[:, cv2.CC_STAT_AREA] / float(d.camera_width * d.camera_height)
        idx = [i for i, v in enumerate(occ) if v > 0.001]
        if not idx: return None, None
        i = max(idx, key=lambda k: occ[k]); s = st[i]; c = ct[i]
        return [int(s[cv2.CC_STAT_LEFT]), int(s[cv2.CC_STAT_TOP]), int(s[cv2.CC_STAT_WIDTH]), int(s[cv2.CC_STAT_HEIGHT])], [int(c[0]), int(c[1])]

    def _annotate(self, frame, bbox, centroid, s):
        h, w = frame.shape[:2]
        cv2.line(frame, (w // 2, 0), (w // 2, h - 1), (255, 255, 0), 1)
        if bbox is not None:
            x, y, bw, bh = bbox; cv2.rectangle(frame, (x, y), (x + bw, y + bh), (0, 255, 0), 2)
        if centroid is not None:
            cv2.circle(frame, (centroid[0], centroid[1]), 5, (0, 255, 255), -1)
            cv2.putText(frame, f'centroid=({centroid[0]},{centroid[1]})', (10, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 2)
        cv2.putText(frame, f"phase={s['phase']} prob={s['cone_probability']:.2f} dir={s['cone_direction']:.2f}", (10, h - 28), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 2)
        if s['cone_is_reached'] or s['phase'] == 6:
            cv2.rectangle(frame, (0, 0), (w - 1, 40), (0, 140, 0), -1)
            cv2.putText(frame, 'GOAL REACHED', (10, 28), cv2.FONT_HERSHEY_DUPLEX, 0.9, (255, 255, 255), 2)
        return frame
    def cone_detect(self):
        if self.detector is None:
            if self.camera_dead_since is None: self.camera_dead_since = time.time()
            self._try_reinit_camera()
            with self.state.lock:
                self.state.cone_direction = CONE_CENTER_POSITION
                self.state.cone_probability = 0.0
                self.state.cone_is_reached = False
                self.state.camera_debug = {'detected': False, 'centroid_px': None, 'bbox_px': None, 'goal_sign': self.state.phase == 6, 'message': 'detector_unavailable'}
            return
        try:
            self.detector.detect_cone()
            prob = self.detector.probability if self.detector.probability else 0.0
            cdir = 1.0 - self.detector.cone_direction if self.detector.cone_direction is not None else CONE_CENTER_POSITION
            bbox, centroid = self._extract_box()
            frame_b64 = None
            if self.detector.input_img is not None:
                s = self.state.snapshot()
                f = self._annotate(self.detector.input_img.copy(), bbox, centroid, s)
                f = cv2.resize(f, (320, 240), interpolation=cv2.INTER_LINEAR)
                ok, enc = cv2.imencode('.jpg', f, [int(cv2.IMWRITE_JPEG_QUALITY), int(self.args.jpeg_quality)])
                if ok: frame_b64 = base64.b64encode(enc.tobytes()).decode('ascii')
            with self.state.lock:
                self.state.cone_direction = float(cdir)
                self.state.cone_probability = float(prob)
                self.state.cone_is_reached = bool(self.detector.is_reached)
                self.state.camera_debug = {'detected': bool(self.detector.is_detected), 'centroid_px': centroid, 'bbox_px': bbox, 'goal_sign': bool(self.detector.is_reached or self.state.phase == 6), 'message': 'ok'}
                if frame_b64 is not None:
                    self.state.frame_b64 = frame_b64
                    self.state.frame_seq += 1
            self.camera_fail_count = 0; self.camera_dead_since = None
        except Exception as e:
            self.camera_fail_count += 1
            if self.camera_fail_count >= CAMERA_FAIL_LIMIT:
                self.detector = None
                if self.camera_dead_since is None: self.camera_dead_since = time.time()
            with self.state.lock:
                self.state.cone_direction = CONE_CENTER_POSITION
                self.state.cone_probability = 0.0
                self.state.cone_is_reached = False
                self.state.camera_debug = {'detected': False, 'centroid_px': None, 'bbox_px': None, 'goal_sign': self.state.phase == 6, 'message': str(e)}

    def handle_phase4(self):
        lr, lg = self.devices.get('led_red'), self.devices.get('led_green')
        s = self.state.snapshot(); cone_prob = s['cone_probability']
        if lr: lr.off()
        if lg: lg.on()
        if not self.searching_flag:
            self.searching_flag = True; self.time_start_searching_cone = time.time(); self.camera_phase4_attempts += 1; self.camera_phase4_start = self.time_start_searching_cone
        elif time.time() - self.time_start_searching_cone >= TIMEOUT_PHASE_4:
            self.searching_flag = False
            with self.state.lock: self.state.phase = 5
            self.time_camera_start = time.time()
        dead = self.camera_dead_since is not None and time.time() - self.camera_dead_since >= CAMERA_DEAD_TIMEOUT
        if dead and (self.camera_phase4_attempts >= CAMERA_PHASE4_MAX_ATTEMPTS or (self.camera_phase4_start is not None and time.time() - self.camera_phase4_start >= TIMEOUT_PHASE_4)):
            return
        if cone_prob > CONE_PROBABILITY_THRESHOLD:
            with self.state.lock: self.state.phase = 5

    def handle_phase5(self):
        lr, lg = self.devices.get('led_red'), self.devices.get('led_green')
        if self.time_camera_start == 0:
            self.time_camera_start = time.time(); self.count_cone_lost = 0; self.camera_phase5_attempts += 1; self.camera_phase5_start = self.time_camera_start
        self.led_blink_timer += 1
        if (self.led_blink_timer // LED_INTERVAL_PHASE5) % 2 == 0:
            if lr: lr.on()
            if lg: lg.off()
        else:
            if lr: lr.off()
            if lg: lg.on()
        s = self.state.snapshot(); is_det = s['cone_probability'] > CONE_PROBABILITY_THRESHOLD; is_reach = s['cone_is_reached']
        dead = self.camera_dead_since is not None and time.time() - self.camera_dead_since >= CAMERA_DEAD_TIMEOUT
        if dead and (self.camera_phase5_attempts >= CAMERA_PHASE5_MAX_ATTEMPTS or (self.camera_phase5_start is not None and time.time() - self.camera_phase5_start >= TIMEOUT_PHASE_5)):
            with self.state.lock: self.state.phase = 4
            self.time_camera_start = 0; return
        self.count_cone_lost = self.count_cone_lost + 1 if not is_det else 0
        if self.count_cone_lost >= CONE_LOST_COUNT_LIMIT:
            with self.state.lock: self.state.phase = 4
            self.time_camera_start = 0; return
        if time.time() - self.time_camera_start >= TIMEOUT_PHASE_5:
            with self.state.lock: self.state.phase = 6
            return
        if is_reach:
            with self.state.lock: self.state.phase = 6

    def handle_phase6(self):
        lr, lg = self.devices.get('led_red'), self.devices.get('led_green')
        if lr: lr.on()
        if lg: lg.on()
        self.stop_motors()
        if self.args.exit_on_goal: self.stop.set()

    def _phase_loop(self):
        while not self.stop.is_set():
            p = self.state.snapshot()['phase']
            if p == 4: self.handle_phase4()
            elif p == 5: self.handle_phase5()
            elif p == 6: self.handle_phase6()
            time.sleep(0.1)

    def _camera_loop(self):
        while not self.stop.is_set():
            p = self.state.snapshot()['phase']
            if p in [4, 5, 6]: self.cone_detect(); time.sleep(CAMERA_ACTIVE_SLEEP)
            else: time.sleep(CAMERA_IDLE_SLEEP)

    def _bno_loop(self):
        while not self.stop.is_set():
            if self.bno is None: time.sleep(1.0); continue
            try:
                acc, gyro, mag, eul = self.bno.getAcc(), self.bno.getGyro(), self.bno.getMag(), self.bno.getEuler()
                angle = float(eul['value'][0]) if eul['valid'] and len(eul['value']) >= 1 else 0.0
                accv = list(acc['value']); gyrov = list(gyro['value']); magv = list(mag['value'])
                fall = math.sqrt(accv[0] ** 2 + accv[1] ** 2 + accv[2] ** 2)
                with self.state.lock:
                    self.state.acc, self.state.gyro, self.state.mag = accv, gyrov, magv
                    self.state.fall, self.state.angle = fall, angle
                    self.state.angle_valid = bool(eul['valid'])
                    self.state.bno_stale_sec = 0.0
                    self.bno_last_valid_time = time.time()
            except Exception:
                with self.state.lock:
                    self.state.angle_valid = False
                    self.state.bno_stale_sec = time.time() - self.bno_last_valid_time if self.bno_last_valid_time > 0 else 0.0
            time.sleep(0.06)
    def _gps_loop(self):
        def probe(ser, sec=2.0):
            st = time.time()
            while time.time() - st < sec:
                b = ser.readline()
                if b and b.decode('utf-8', errors='ignore').strip().startswith('$'): return True
            return False
        def open_serial():
            ports = [GPS_SERIAL_PORT] + [p for p in GPS_SERIAL_PORT_CANDIDATES if p != GPS_SERIAL_PORT]
            bauds = [GPS_BAUDRATE] + [b for b in GPS_BAUDRATE_CANDIDATES if b != GPS_BAUDRATE]
            for p in ports:
                for b in bauds:
                    try:
                        s = serial.Serial(p, b, timeout=0.2)
                        try: s.reset_input_buffer()
                        except Exception: pass
                        if probe(s): s.timeout = GPS_SERIAL_TIMEOUT; return s
                        s.close()
                    except Exception:
                        pass
            return None
        ser = open_serial(); lbc = time.time(); lft = 0.0; lvft = 0.0; lvl = None; stable = 0
        while not self.stop.is_set():
            try:
                if ser is None or not ser.is_open: ser = open_serial(); time.sleep(1.0); continue
                now = time.time()
                if lvft > 0 and now - lvft > GPS_FIX_LOSS_TIMEOUT:
                    stable = 0
                    with self.state.lock: self.state.gps_detect = 0; self.state.gps_heading_valid = False
                if ser.in_waiting > GPS_BUFFER_CLEAR_THRESHOLD and now - lbc >= GPS_BUFFER_CLEAR_INTERVAL:
                    try: ser.reset_input_buffer()
                    except Exception: pass
                    lbc = now; stable = 0
                lb = ser.readline()
                if not lb: continue
                line = lb.decode('utf-8', errors='ignore').strip()
                if not (line.startswith('$GPGGA') or line.startswith('$GNGGA')): continue
                try: msg = pynmea2.parse(line, check=True)
                except Exception: continue
                if getattr(msg, 'sentence_type', '') != 'GGA': continue
                latv, lngv = getattr(msg, 'latitude', None), getattr(msg, 'longitude', None)
                if latv is None or lngv is None: continue
                lat, lng = float(latv), float(lngv)
                gq, ns, hd = getattr(msg, 'gps_qual', None), getattr(msg, 'num_sats', None), getattr(msg, 'horizontal_dil', None)
                try: qok = gq is not None and int(gq) >= GPS_MIN_FIX_QUAL
                except Exception: qok = False
                try: sok = ns is not None and int(ns) >= GPS_MIN_SATELLITES
                except Exception: sok = False
                try: hok = hd is not None and float(hd) <= GPS_MAX_HDOP
                except Exception: hok = True
                if not (qok and sok and hok and (lat != 0.0 or lng != 0.0)): stable = 0; continue
                if lvl is not None:
                    dist, _ = calc_distance_and_azimuth(lvl[0], lvl[1], lat, lng)
                    dt = now - lft if lft > 0 else 0
                    if dt > 0 and dist / dt > GPS_MAX_SPEED_MPS: stable = 0; continue
                stable += 1; lft = now
                if stable >= GPS_STABLE_FIX_COUNT:
                    gh = None; ghv = False
                    if lvl is not None:
                        dist, course = calc_distance_and_azimuth(lvl[0], lvl[1], lat, lng)
                        if dist >= GPS_HEADING_MIN_DIST: gh, ghv = course, True
                    with self.state.lock:
                        self.state.lat, self.state.lng = lat, lng
                        self.state.gps_detect, self.state.gps_heading, self.state.gps_heading_valid = 1, gh, ghv
                        self.state.num_sats = int(ns) if ns is not None else None
                        self.state.hdop = float(hd) if hd not in (None, '') else None
                        self.state.gps_qual = int(gq) if gq is not None else None
                    lvft = now; lvl = (lat, lng)
                else:
                    with self.state.lock: self.state.gps_detect = 0; self.state.gps_heading_valid = False
            except Exception:
                try:
                    if ser is not None: ser.close()
                except Exception:
                    pass
                ser = None; time.sleep(1.0)

    def _ramp2(self, pa, sa, ta, pb, sb, tb, rt, si=MOTOR_RAMP_STEP):
        if pa is None and pb is None: return ta, tb
        if rt <= 0 or si <= 0:
            if pa is not None: pa.value = max(0.0, min(1.0, ta / 100.0))
            if pb is not None: pb.value = max(0.0, min(1.0, tb / 100.0))
            return ta, tb
        st = max(1, int(rt / si)); sd = rt / st
        for i in range(1, st + 1):
            da, db = sa + (ta - sa) * (i / st), sb + (tb - sb) * (i / st)
            if pa is not None: pa.value = max(0.0, min(1.0, da / 100.0))
            if pb is not None: pb.value = max(0.0, min(1.0, db / 100.0))
            time.sleep(sd)
        return ta, tb

    def set_motors(self, sa, fa, sb, fb, rt=MOTOR_RAMP_TIME, si=MOTOR_RAMP_STEP):
        p1, d1 = self.devices.get('motor_1_pwm'), self.devices.get('motor_1_dir')
        p2, d2 = self.devices.get('motor_2_pwm'), self.devices.get('motor_2_dir')
        if p1 is None or d1 is None or p2 is None or d2 is None: return
        a, b = self.motor_state.setdefault(p1, {'speed': 0.0, 'direction': True}), self.motor_state.setdefault(p2, {'speed': 0.0, 'direction': True})
        ca, cb = a['speed'], b['speed']
        if (ca > 0 and fa != a['direction']) or (cb > 0 and fb != b['direction']):
            ca, cb = self._ramp2(p1, ca, 0, p2, cb, 0, rt / 2, si)
        d1.value = 1 if (fa ^ MOTOR_DIR_INVERT_1) else 0
        d2.value = 1 if (fb ^ MOTOR_DIR_INVERT_2) else 0
        ta, tb = max(0.0, min(100.0, sa)), max(0.0, min(100.0, sb))
        ca, cb = self._ramp2(p1, ca, ta, p2, cb, tb, rt, si)
        a['speed'], a['direction'], b['speed'], b['direction'] = ca, fa, cb, fb

    def stop_motors(self):
        p1, p2 = self.devices.get('motor_1_pwm'), self.devices.get('motor_2_pwm')
        if p1: p1.value = 0
        if p2: p2.value = 0
        for s in self.motor_state.values(): s['speed'] = 0.0

    def _motor_loop(self):
        while not self.stop.is_set():
            s = self.state.snapshot(); p = s['phase']
            if p == 6: self.stop_motors(); time.sleep(0.1); continue
            if p == 4: self.set_motors(SEARCH_ROTATION_SPEED, True, SEARCH_ROTATION_SPEED, False)
            elif p == 5:
                err = s['cone_direction'] - CONE_CENTER_POSITION
                t = err * APPROACH_TURN_GAIN
                sl, sr = max(0, min(100, BASE_SPEED + t)), max(0, min(100, BASE_SPEED - t))
                self.set_motors(sr, True, sl, True)
            time.sleep(MOTOR_LOOP_INTERVAL)

    def _send(self, sock, payload):
        b = json.dumps(payload, ensure_ascii=True, separators=(',', ':')).encode('utf-8')
        sock.sendall(struct.pack('>I', len(b))); sock.sendall(b)

    def _tx_loop(self):
        c = 0; dt = 1.0 / max(1.0, self.args.tx_hz)
        while not self.stop.is_set():
            sock = None
            try:
                sock = socket.create_connection((self.args.pc_host, self.args.pc_port), timeout=5.0)
                sock.settimeout(5.0)
                while not self.stop.is_set():
                    s = self.state.snapshot(); inc = c % max(1, self.args.video_every) == 0
                    p = {'type': 'telemetry', 'timestamp': time.time(), 'phase': s['phase'],
                         'camera': {'cone_probability': s['cone_probability'], 'cone_direction': s['cone_direction'], 'cone_is_reached': s['cone_is_reached'], 'debug': s['camera_debug']},
                         'bno': {'acc': s['acc'], 'gyro': s['gyro'], 'mag': s['mag'], 'angle': s['angle'], 'angle_valid': s['angle_valid'], 'fall': s['fall'], 'stale_sec': s['bno_stale_sec']},
                         'gps': {'lat': s['lat'], 'lng': s['lng'], 'gps_detect': s['gps_detect'], 'gps_heading': s['gps_heading'], 'gps_heading_valid': s['gps_heading_valid'], 'num_sats': s['num_sats'], 'hdop': s['hdop'], 'gps_qual': s['gps_qual']},
                         'frame_seq': s['frame_seq']}
                    if inc: p['frame_jpeg_b64'] = s['frame_b64']
                    self._send(sock, p); c += 1; time.sleep(dt)
            except Exception:
                time.sleep(1.0)
            finally:
                if sock is not None:
                    try: sock.close()
                    except Exception: pass

    def run(self):
        self.setup()
        ths = [threading.Thread(target=t, daemon=True) for t in [self._phase_loop, self._camera_loop, self._gps_loop, self._bno_loop, self._motor_loop, self._tx_loop]]
        [t.start() for t in ths]
        try:
            while not self.stop.is_set(): time.sleep(1.0)
        except KeyboardInterrupt:
            self.stop.set()
        finally:
            self.stop_motors()


def parse_args():
    p = argparse.ArgumentParser(description='SBC debug relay for phase4-6 camera + motor + LED + sensors')
    p.add_argument('--pc-host', required=True)
    p.add_argument('--pc-port', type=int, default=5001)
    p.add_argument('--jpeg-quality', type=int, default=55)
    p.add_argument('--tx-hz', type=float, default=10.0)
    p.add_argument('--video-every', type=int, default=2)
    p.add_argument('--start-phase', type=int, default=4, choices=[4, 5, 6])
    p.add_argument('--exit-on-goal', action='store_true')
    return p.parse_args()


def main():
    a = parse_args()
    a.jpeg_quality = max(1, min(95, a.jpeg_quality)); a.tx_hz = max(1.0, a.tx_hz); a.video_every = max(1, a.video_every)
    Relay(a).run()


if __name__ == '__main__':
    main()
