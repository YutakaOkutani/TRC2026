import math
import time

from csmn.const import DEGREE_FULL_CIRCLE, EARTH_RADIUS_METERS, MILLISECOND_SCALE


def current_milli_time():
    return round(time.time() * MILLISECOND_SCALE)


def calc_distance_and_azimuth(lat1, lng1, lat2, lng2):
    rad_lat1 = math.radians(lat1)
    rad_lng1 = math.radians(lng1)
    rad_lat2 = math.radians(lat2)
    rad_lng2 = math.radians(lng2)
    d_lng = rad_lng2 - rad_lng1
    sin_lat1 = math.sin(rad_lat1)
    cos_lat1 = math.cos(rad_lat1)
    sin_lat2 = math.sin(rad_lat2)
    cos_lat2 = math.cos(rad_lat2)
    cos_d_lng = math.cos(d_lng)
    val = sin_lat1 * sin_lat2 + cos_lat1 * cos_lat2 * cos_d_lng
    val = max(-1.0, min(1.0, val))
    central_angle = math.acos(val)
    dist = EARTH_RADIUS_METERS * central_angle
    y = math.sin(d_lng) * cos_lat2
    x = cos_lat1 * sin_lat2 - sin_lat1 * cos_lat2 * cos_d_lng
    azi = math.degrees(math.atan2(y, x))
    if azi < 0.0:
        azi += DEGREE_FULL_CIRCLE
    return dist, azi
