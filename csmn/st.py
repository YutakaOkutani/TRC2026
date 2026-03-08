import threading

from csmn.const import (
    CONE_CENTER_POSITION,
    DEFAULT_FLOAT_VALUE,
    DEFAULT_OBSTACLE_DIST_CM,
    DEFAULT_PHASE,
    DEFAULT_VECTOR3,
)


class CanSatState:
    def __init__(self):
        self.lock = threading.Lock()
        self.acc = list(DEFAULT_VECTOR3)
        self.gyro = list(DEFAULT_VECTOR3)
        self.mag = list(DEFAULT_VECTOR3)
        self.lat = DEFAULT_FLOAT_VALUE
        self.lng = DEFAULT_FLOAT_VALUE
        self.gps_heading = DEFAULT_FLOAT_VALUE
        self.gps_heading_valid = False
        self.gps_speed_mps = DEFAULT_FLOAT_VALUE
        self.gps_fix_qual = 0
        self.gps_sats = 0
        self.gps_hdop = DEFAULT_FLOAT_VALUE
        self.alt = DEFAULT_FLOAT_VALUE
        self.pres = DEFAULT_FLOAT_VALUE
        self.distance = DEFAULT_FLOAT_VALUE
        self.azimuth = DEFAULT_FLOAT_VALUE
        self.angle = DEFAULT_FLOAT_VALUE
        self.angle_valid = False
        self.direction = DEFAULT_FLOAT_VALUE
        self.fall = DEFAULT_FLOAT_VALUE
        self.cone_direction = CONE_CENTER_POSITION
        self.cone_probability = DEFAULT_FLOAT_VALUE
        self.cone_method = ""
        self.obstacle_dist = DEFAULT_OBSTACLE_DIST_CM
        self.phase = DEFAULT_PHASE
        self.gps_detect = 0
        self.cone_is_reached = False

    def update_imu(self, acc=None, gyro=None, mag=None, fall=None, angle=None, angle_valid=None):
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
            if angle_valid is not None:
                self.angle_valid = angle_valid

    def update_gps(
        self,
        lat=None,
        lng=None,
        gps_detect=None,
        gps_heading=None,
        gps_heading_valid=None,
        gps_speed_mps=None,
        gps_fix_qual=None,
        gps_sats=None,
        gps_hdop=None,
    ):
        with self.lock:
            if lat is not None:
                self.lat = lat
            if lng is not None:
                self.lng = lng
            if gps_detect is not None:
                self.gps_detect = gps_detect
            if gps_heading is not None:
                self.gps_heading = gps_heading
            if gps_heading_valid is not None:
                self.gps_heading_valid = gps_heading_valid
            if gps_speed_mps is not None:
                self.gps_speed_mps = gps_speed_mps
            if gps_fix_qual is not None:
                self.gps_fix_qual = gps_fix_qual
            if gps_sats is not None:
                self.gps_sats = gps_sats
            if gps_hdop is not None:
                self.gps_hdop = gps_hdop

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

    def update_cone(self, cone_direction=None, cone_probability=None, cone_is_reached=None, cone_method=None):
        with self.lock:
            if cone_direction is not None:
                self.cone_direction = cone_direction
            if cone_probability is not None:
                self.cone_probability = cone_probability
            if cone_is_reached is not None:
                self.cone_is_reached = cone_is_reached
            if cone_method is not None:
                self.cone_method = cone_method

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
                "gps_heading": self.gps_heading,
                "gps_heading_valid": self.gps_heading_valid,
                "gps_speed_mps": self.gps_speed_mps,
                "gps_fix_qual": self.gps_fix_qual,
                "gps_sats": self.gps_sats,
                "gps_hdop": self.gps_hdop,
                "alt": self.alt,
                "pres": self.pres,
                "distance": self.distance,
                "azimuth": self.azimuth,
                "angle": self.angle,
                "angle_valid": self.angle_valid,
                "direction": self.direction,
                "fall": self.fall,
                "cone_direction": self.cone_direction,
                "cone_probability": self.cone_probability,
                "cone_method": self.cone_method,
                "obstacle_dist": self.obstacle_dist,
                "phase": self.phase,
                "gps_detect": self.gps_detect,
                "cone_is_reached": self.cone_is_reached,
            }
