import math
import time

from cansat_mission.constants import (
    APPROACH_TURN_GAIN,
    BASE_SPEED,
    CONE_CENTER_POSITION,
    CONE_PROBABILITY_THRESHOLD,
    CONE_PROBABILITY_THRESHOLD_PHASE4,
    CONE_PROBABILITY_THRESHOLD_PHASE5,
    DEVICE_MOTOR_1_DIR,
    DEVICE_MOTOR_1_PWM,
    DEVICE_MOTOR_2_DIR,
    DEVICE_MOTOR_2_PWM,
    MOTOR_DIR_INVERT_1,
    MOTOR_DIR_INVERT_2,
    MANUAL_TURN_SPEED_RATIO,
    MOTOR_IDLE_SLEEP,
    MOTOR_LOOP_INTERVAL,
    MOTOR_RAMP_STEP,
    MOTOR_RAMP_TIME,
    MOTOR_SPEED_OFFSET_1,
    MOTOR_SPEED_OFFSET_2,
    MOTOR_SPEED_SCALE_1,
    MOTOR_SPEED_SCALE_2,
    OBSTACLE_AVOID_DIST,
    OBSTACLE_BACKUP_TIME,
    OBSTACLE_CONFIRM_COUNT,
    OBSTACLE_PAUSE_TIME,
    OBSTACLE_SPEED,
    OBSTACLE_TURN_TIME,
    PARACHUTE_DIRECTION,
    PARACHUTE_MOTOR_PULSE,
    PARACHUTE_SEPARATION_SPEED,
    PHASE4_SEARCH_SWEEP_INTERVAL,
    PHASE4_ALIGN_FORWARD_SPEED,
    PHASE4_ALIGN_PIVOT_SPEED,
    PHASE4_ALIGN_STOP_DEADBAND,
    PHASE45_CONE_DIR_FILTER_ALPHA,
    PHASE5_BASE_SPEED,
    PHASE5_STEER_DEADBAND,
    PHASE5_TURN_CLAMP,
    PHASE2_SPEED,
    PHASE2_STAGE_STRAIGHT,
    PHASE2_TURN_BIAS,
    PHASE2_TURN_INTERVAL,
    PHASE2_RAMP_TIME,
    PHASE3_FORWARD_RAMP_TIME,
    PHASE3_GPS_FALLBACK_DEADBAND_DEG,
    PHASE3_GPS_FALLBACK_TURN_SCALE,
    PHASE3_HEADING_DEADBAND_DEG,
    PHASE3_TURN_FULL_ERROR_DEG,
    PHASE3_TURN_FAST_SPEED,
    PHASE3_TURN_RAMP_TIME,
    PHASE3_TURN_SLOW_SPEED,
    PHASES_SKIP_OBSTACLE,
    PHASES_STOP_MOTORS,
    PWM_DUTY_MAX,
    PWM_DUTY_MIN,
    PWM_PERCENT_MAX,
    PWM_PERCENT_MIN,
    RAMP_HALF_DIVISOR,
    SEARCH_ROTATION_SPEED,
    TURN_GAIN_SCALE_MAX,
    TURN_GAIN_SCALE_MIN,
    Phase,
)


MANUAL_DRIVE_PATTERNS = {
    "w": ("Forward", True, True),
    "s": ("Backward", False, False),
    "a": ("Left", True, True),
    "d": ("Right", True, True),
}


def get_manual_drive_pattern(cmd, speed):
    """Return a normalized two-motor command for manual WASD control.

    Motor ordering is fixed as:
    - A: MTR1 (left)
    - B: MTR2 (right)
    """
    pattern = MANUAL_DRIVE_PATTERNS.get((cmd or "").lower())
    if pattern is None:
        return None
    label, forward_a, forward_b = pattern
    speed_fast = float(speed)
    speed_slow = speed_fast * MANUAL_TURN_SPEED_RATIO
    speed_a = speed_fast
    speed_b = speed_fast
    cmd_key = (cmd or "").lower()
    if cmd_key == "a":
        speed_a = speed_slow
    elif cmd_key == "d":
        speed_b = speed_slow
    return {
        "label": label,
        "speed_a": speed_a,
        "forward_a": bool(forward_a),
        "speed_b": speed_b,
        "forward_b": bool(forward_b),
    }


class MotorManager:
    def _shutdown_active(self):
        return bool(getattr(self, "_shutdown_requested", False))

    def _set_forward_diff_turn(self, fast_side, speed_fast, speed_slow, cmd_type, ramp_time=MOTOR_RAMP_TIME):
        """Steer with differential forward speeds only (no reverse)."""
        speed_fast = self._clamp_percent(speed_fast)
        speed_slow = self._clamp_percent(speed_slow)
        if fast_side == "left":
            self.set_motors(speed_fast, True, speed_slow, True, ramp_time=ramp_time, cmd_type=cmd_type)
        else:
            self.set_motors(speed_slow, True, speed_fast, True, ramp_time=ramp_time, cmd_type=cmd_type)

    def _set_forward_pivot_turn(self, turn_side, speed_outer, cmd_type, speed_inner=0.0):
        """Turn with one wheel driving forward and the inner wheel stopped."""
        speed_outer = self._clamp_percent(speed_outer)
        speed_inner = self._clamp_percent(speed_inner)
        if turn_side == "left":
            self.set_motors(speed_inner, True, speed_outer, True, cmd_type=cmd_type)
        else:
            self.set_motors(speed_outer, True, speed_inner, True, cmd_type=cmd_type)

    def _mag_heading_from_snapshot(self, snapshot):
        mag = snapshot.get("mag")
        if not isinstance(mag, (list, tuple)) or len(mag) < 2:
            return None
        try:
            mx = float(mag[0])
            my = float(mag[1])
        except (TypeError, ValueError):
            return None
        if not math.isfinite(mx) or not math.isfinite(my):
            return None
        if abs(mx) < 1e-6 and abs(my) < 1e-6:
            return None
        # Legacy rover logic (main(7).py): magnetometer XY -> heading.
        azimuth = 90.0 - math.degrees(math.atan2(my, mx))
        azimuth *= -1.0
        azimuth %= 360.0
        return azimuth

    def _phase3_heading(self, snapshot):
        # Phase3 heading policy (legacy-aligned with main(8).py):
        # 1) Prefer magnetometer heading (azimuth from BNO mag XY).
        # 2) Fall back to Euler heading (BNO), optionally GPS-aligned.
        # 3) Fall back to GPS-derived course only when IMU heading is unavailable.
        mag_heading = self._mag_heading_from_snapshot(snapshot)
        if mag_heading is not None:
            if snapshot.get("gps_heading_valid", False) and hasattr(self, "_update_mag_heading_offset_from_gps"):
                self._update_mag_heading_offset_from_gps(snapshot, snapshot.get("gps_heading"), mag_heading)
            if hasattr(self, "_mag_heading_aligned_to_gps"):
                aligned = self._mag_heading_aligned_to_gps(mag_heading)
                if aligned is not None:
                    source = "MAG_ALIGNED" if getattr(self, "mag_heading_offset_valid", False) else "MAG"
                    return aligned, source
            return mag_heading, "MAG"

        try:
            angle = float(snapshot.get("angle", 0.0))
        except (TypeError, ValueError):
            angle = 0.0
        if snapshot.get("angle_valid", False) and math.isfinite(angle):
            if snapshot.get("gps_heading_valid", False) and hasattr(self, "_update_bno_heading_offset_from_gps"):
                self._update_bno_heading_offset_from_gps(snapshot)
            if hasattr(self, "_bno_heading_aligned_to_gps"):
                aligned = self._bno_heading_aligned_to_gps(snapshot)
                if aligned is not None:
                    source = "BNO_ALIGNED" if getattr(self, "bno_heading_offset_valid", False) else "BNO"
                    return aligned, source
            return angle, "BNO"
        if snapshot.get("gps_heading_valid", False):
            gps_heading = self._normalize_heading_deg(snapshot.get("gps_heading"))
            if gps_heading is not None:
                return gps_heading, "GPS_FALLBACK"
        if snapshot.get("angle_valid", False):
            return None, "BNO_PARSE_FAIL"
        if snapshot.get("gps_heading_valid", False):
            return None, "GPS_PARSE_FAIL"
        return None, "NO_HEADING_SOURCE"

    def _clamp_percent(self, value):
        return max(PWM_PERCENT_MIN, min(PWM_PERCENT_MAX, value))

    def _get_motor_speed_scale(self, motor_index):
        default = MOTOR_SPEED_SCALE_1 if motor_index == 1 else MOTOR_SPEED_SCALE_2
        attr_name = f"motor_speed_scale_{motor_index}"
        try:
            scale = float(getattr(self, attr_name, default))
        except (TypeError, ValueError):
            scale = default
        return max(0.0, scale)

    def _get_motor_speed_offset(self, motor_index):
        default = MOTOR_SPEED_OFFSET_1 if motor_index == 1 else MOTOR_SPEED_OFFSET_2
        attr_name = f"motor_speed_offset_{motor_index}"
        try:
            offset = float(getattr(self, attr_name, default))
        except (TypeError, ValueError):
            offset = default
        return offset

    def _apply_motor_speed_scale(self, speed, motor_index):
        scaled = float(speed) * self._get_motor_speed_scale(motor_index)
        adjusted = scaled + self._get_motor_speed_offset(motor_index)
        return self._clamp_percent(adjusted)

    def _phase45_bno_heading(self, snapshot):
        heading, _ = self._phase3_heading(snapshot)
        return heading

    def _phase3_turn_speeds(self, diff_deg, deadband_deg=PHASE3_HEADING_DEADBAND_DEG, turn_scale=1.0):
        base = self._clamp_percent(BASE_SPEED)
        fast = self._clamp_percent(PHASE3_TURN_FAST_SPEED)
        slow = self._clamp_percent(PHASE3_TURN_SLOW_SPEED)
        abs_diff = abs(float(diff_deg))
        deadband = max(0.0, float(deadband_deg))
        full_error = max(deadband + 1.0, float(PHASE3_TURN_FULL_ERROR_DEG))
        scale = max(0.0, min(1.0, float(turn_scale)))
        effective_fast = base + (fast - base) * scale
        effective_slow = base - (base - slow) * scale
        if abs_diff <= deadband:
            return base, base
        ratio = min(1.0, max(0.0, (abs_diff - deadband) / (full_error - deadband)))
        outer_speed = self._clamp_percent(base + (effective_fast - base) * ratio)
        inner_speed = self._clamp_percent(base - (base - effective_slow) * ratio)
        return outer_speed, inner_speed

    def _reset_phase45_camera_track(self):
        self.phase45_filtered_cone_dir = None
        self.phase45_last_seen_time = None

    def _update_phase45_filtered_cone_dir(self, raw_cone_direction, cone_seen):
        if not cone_seen:
            return getattr(self, "phase45_filtered_cone_dir", None)
        try:
            cdir = float(raw_cone_direction)
        except (TypeError, ValueError):
            return getattr(self, "phase45_filtered_cone_dir", None)
        cdir = max(0.0, min(1.0, cdir))
        prev = getattr(self, "phase45_filtered_cone_dir", None)
        if prev is None:
            filtered = cdir
        else:
            alpha = PHASE45_CONE_DIR_FILTER_ALPHA
            filtered = (1.0 - alpha) * float(prev) + alpha * cdir
        self.phase45_filtered_cone_dir = filtered
        self.phase45_last_seen_time = time.time()
        return filtered

    def _record_motor_command(self, cmd_type, motor1_speed, motor1_forward, motor2_speed, motor2_forward):
        self.last_motor_command = {
            "type": cmd_type,
            "updated_ms": int(time.time() * 1000),
            "motor1_speed": float(motor1_speed),
            "motor1_forward": int(bool(motor1_forward)),
            "motor2_speed": float(motor2_speed),
            "motor2_forward": int(bool(motor2_forward)),
        }

    def move_motor_thread(self):
        while not self._shutdown_active():
            snapshot = self.state.snapshot()
            phase = Phase(snapshot["phase"])
            obstacle_dist = snapshot["obstacle_dist"]
            direction = snapshot["direction"]
            cone_direction = snapshot["cone_direction"]
            last_phase = getattr(self, "_motor_last_phase", None)
            if last_phase != phase:
                if phase not in (Phase.PHASE4, Phase.PHASE5) or last_phase not in (Phase.PHASE4, Phase.PHASE5):
                    self._reset_phase45_camera_track()
                self._motor_last_phase = phase

            if phase in PHASES_STOP_MOTORS:
                self.stop_motors()
                time.sleep(MOTOR_IDLE_SLEEP)
                continue

            obstacle_detected = (
                phase not in PHASES_SKIP_OBSTACLE
                and obstacle_dist is not None
                and 0 < obstacle_dist < OBSTACLE_AVOID_DIST
            )
            if obstacle_detected:
                self.obstacle_detect_count += 1
            else:
                self.obstacle_detect_count = 0

            if self.obstacle_detect_count >= OBSTACLE_CONFIRM_COUNT:
                print(f"Obstacle Detected! {obstacle_dist:.1f}cm")
                self.stop_motors()
                time.sleep(OBSTACLE_PAUSE_TIME)
                turn_fast = self._clamp_percent(OBSTACLE_SPEED)
                turn_slow = self._clamp_percent(OBSTACLE_SPEED * 0.35)
                self._set_forward_diff_turn("right", turn_fast, turn_slow, cmd_type="obstacle_forward_turn")
                time.sleep(OBSTACLE_TURN_TIME)
                self.stop_motors()
                time.sleep(OBSTACLE_PAUSE_TIME)
                self.obstacle_detect_count = 0
                continue

            if phase == Phase.PHASE1 and direction == PARACHUTE_DIRECTION:
                self.set_motors(
                    PARACHUTE_SEPARATION_SPEED,
                    True,
                    PARACHUTE_SEPARATION_SPEED,
                    True,
                    ramp_time=0.0,
                    cmd_type="phase1_parachute_separation",
                )
                time.sleep(PARACHUTE_MOTOR_PULSE)
                continue

            if phase == Phase.PHASE2:
                if self.phase2_stage == PHASE2_STAGE_STRAIGHT:
                    self.set_motors(
                        PHASE2_SPEED,
                        True,
                        PHASE2_SPEED,
                        True,
                        ramp_time=PHASE2_RAMP_TIME,
                        cmd_type="phase2_straight",
                    )
                else:
                    elapsed = 0.0
                    if self.phase2_stage_start is not None:
                        elapsed = time.time() - self.phase2_stage_start
                    left_turn = int(elapsed // PHASE2_TURN_INTERVAL) % 2 == 0
                    bias = self._clamp_percent(PHASE2_TURN_BIAS)
                    base = self._clamp_percent(PHASE2_SPEED)
                    if left_turn:
                        speed_l = self._clamp_percent(base - bias)
                        speed_r = self._clamp_percent(base + bias)
                    else:
                        speed_l = self._clamp_percent(base + bias)
                        speed_r = self._clamp_percent(base - bias)
                    self.set_motors(
                        speed_l,
                        True,
                        speed_r,
                        True,
                        ramp_time=PHASE2_RAMP_TIME,
                        cmd_type="phase2_fig8",
                    )
                time.sleep(MOTOR_LOOP_INTERVAL)
                continue

            if phase == Phase.PHASE3:
                target_heading = direction
                nav_heading, heading_source = self._phase3_heading(snapshot)
                if nav_heading is not None:
                    self.phase3_no_heading_start = None
                    if heading_source != "GPS_FALLBACK":
                        self.phase3_gps_fallback_turn_sign = 0
                        self.phase3_gps_fallback_turn_until = 0.0
                        self.phase3_gps_fallback_cooldown_until = 0.0
                    diff = self._angle_diff_deg(target_heading, nav_heading)
                    heading_deadband = PHASE3_HEADING_DEADBAND_DEG
                    turn_scale = 1.0
                    if heading_source == "GPS_FALLBACK":
                        # main(8).py never used GPS course-over-ground as a continuous
                        # heading source. Treat it only as an occasional coarse nudge;
                        # otherwise it causes large serpentine arcs at low rover speeds.
                        heading_deadband = max(
                            heading_deadband,
                            PHASE3_GPS_FALLBACK_DEADBAND_DEG,
                            PHASE3_TURN_FULL_ERROR_DEG,
                        )
                        turn_scale = PHASE3_GPS_FALLBACK_TURN_SCALE
                        now = time.time()
                        burst_until = float(getattr(self, "phase3_gps_fallback_turn_until", 0.0))
                        cooldown_until = float(getattr(self, "phase3_gps_fallback_cooldown_until", 0.0))
                        active_sign = int(getattr(self, "phase3_gps_fallback_turn_sign", 0))
                        requested_sign = 1 if diff > 0 else -1

                        if abs(diff) <= heading_deadband:
                            self.phase3_gps_fallback_turn_sign = 0
                            self.phase3_gps_fallback_turn_until = 0.0
                        elif now < burst_until:
                            if active_sign != requested_sign:
                                base = self._clamp_percent(BASE_SPEED)
                                self.set_motors(
                                    base,
                                    True,
                                    base,
                                    True,
                                    ramp_time=PHASE3_FORWARD_RAMP_TIME,
                                    cmd_type="phase3_gps_forward",
                                )
                                time.sleep(MOTOR_LOOP_INTERVAL)
                                continue
                        elif now < cooldown_until:
                            base = self._clamp_percent(BASE_SPEED)
                            self.set_motors(
                                base,
                                True,
                                base,
                                True,
                                ramp_time=PHASE3_FORWARD_RAMP_TIME,
                                cmd_type="phase3_gps_forward",
                            )
                            time.sleep(MOTOR_LOOP_INTERVAL)
                            continue
                        else:
                            self.phase3_gps_fallback_turn_sign = requested_sign
                            self.phase3_gps_fallback_turn_until = now + 1.2
                            self.phase3_gps_fallback_cooldown_until = now + 7.0
                    if abs(diff) <= heading_deadband:
                        base = self._clamp_percent(BASE_SPEED)
                        self.set_motors(
                            base,
                            True,
                            base,
                            True,
                            ramp_time=PHASE3_FORWARD_RAMP_TIME,
                            cmd_type="phase3_gps_forward",
                        )
                    else:
                        outer_speed, inner_speed = self._phase3_turn_speeds(
                            diff,
                            deadband_deg=heading_deadband,
                            turn_scale=turn_scale,
                        )
                        if diff > 0:
                            # Need clockwise/right turn: slow right wheel, fast left wheel.
                            self._set_forward_diff_turn(
                                "left",
                                outer_speed,
                                inner_speed,
                                cmd_type="phase3_gps_turn",
                                ramp_time=PHASE3_TURN_RAMP_TIME,
                            )
                        else:
                            # Need counter-clockwise/left turn: slow left wheel, fast right wheel.
                            self._set_forward_diff_turn(
                                "right",
                                outer_speed,
                                inner_speed,
                                cmd_type="phase3_gps_turn",
                                ramp_time=PHASE3_TURN_RAMP_TIME,
                            )
                else:
                    self.phase3_gps_fallback_turn_sign = 0
                    self.phase3_gps_fallback_turn_until = 0.0
                    self.phase3_gps_fallback_cooldown_until = 0.0
                    if self.phase3_no_heading_start is None:
                        self.phase3_no_heading_start = time.time()
                    # Legacy Phase3 was straight-dominant; alternating left/right search here
                    # creates the large S-curves seen in recent logs.
                    base = self._clamp_percent(BASE_SPEED)
                    self.set_motors(
                        base,
                        True,
                        base,
                        True,
                        ramp_time=PHASE3_FORWARD_RAMP_TIME,
                        cmd_type="phase3_no_heading_forward",
                    )
            elif phase == Phase.PHASE4:
                # Phase4 is camera-only: keep all turning forward-only to avoid reverse torque.
                cone_prob = snapshot.get("cone_probability", 0.0)
                cone_reached = snapshot.get("cone_is_reached", False)
                cone_seen = cone_reached or (cone_prob > CONE_PROBABILITY_THRESHOLD_PHASE4)
                if cone_seen:
                    filtered_dir = self._update_phase45_filtered_cone_dir(cone_direction, True)
                    if filtered_dir is None:
                        filtered_dir = CONE_CENTER_POSITION
                    err = filtered_dir - CONE_CENTER_POSITION
                    abs_err = abs(err)
                    if abs_err <= PHASE4_ALIGN_STOP_DEADBAND:
                        self.set_motors(
                            PHASE4_ALIGN_FORWARD_SPEED,
                            True,
                            PHASE4_ALIGN_FORWARD_SPEED,
                            True,
                            cmd_type="phase4_camera_center_forward",
                        )
                        time.sleep(MOTOR_LOOP_INTERVAL)
                        continue
                    turn_side = "right" if err > 0 else "left"
                    self._set_forward_pivot_turn(
                        turn_side,
                        PHASE4_ALIGN_PIVOT_SPEED,
                        cmd_type="phase4_camera_pivot_align",
                    )
                else:
                    self._update_phase45_filtered_cone_dir(cone_direction, False)
                    # Keep rotating in one direction so the rover can sweep through 180 deg
                    # instead of dithering around the initial heading.
                    self._set_forward_pivot_turn(
                        "left",
                        SEARCH_ROTATION_SPEED,
                        cmd_type="phase4_search_pivot",
                    )
            elif phase == Phase.PHASE5:
                cone_prob = snapshot.get("cone_probability", 0.0)
                cone_reached = snapshot.get("cone_is_reached", False)
                cone_seen = cone_reached or (cone_prob > CONE_PROBABILITY_THRESHOLD_PHASE5)
                filtered_dir = self._update_phase45_filtered_cone_dir(cone_direction, cone_seen)
                steer_dir = filtered_dir if filtered_dir is not None else CONE_CENTER_POSITION
                err = steer_dir - CONE_CENTER_POSITION
                if abs(err) <= PHASE5_STEER_DEADBAND:
                    self.set_motors(
                        PHASE5_BASE_SPEED,
                        True,
                        PHASE5_BASE_SPEED,
                        True,
                        cmd_type="phase5_approach_forward",
                    )
                else:
                    turn_total = self._clamp_percent(
                        min(PHASE5_TURN_CLAMP, abs(err) * APPROACH_TURN_GAIN)
                    )
                    inner_speed = self._clamp_percent(
                        max(PHASE5_BASE_SPEED - turn_total, PHASE5_BASE_SPEED * 0.55)
                    )
                    outer_speed = self._clamp_percent(PHASE5_BASE_SPEED + turn_total)
                    if err > 0:
                        self.set_motors(
                            outer_speed,
                            True,
                            inner_speed,
                            True,
                            cmd_type="phase5_approach_steer_right",
                        )
                    else:
                        self.set_motors(
                            inner_speed,
                            True,
                            outer_speed,
                            True,
                            cmd_type="phase5_approach_steer_left",
                        )

            time.sleep(MOTOR_LOOP_INTERVAL)
        self.stop_motors()

    def _ramp_pwm(self, pwm_dev, start_speed, target_speed, ramp_time, step_interval=MOTOR_RAMP_STEP):
        if self._shutdown_active():
            if pwm_dev is not None:
                pwm_dev.value = 0
            return 0.0
        if pwm_dev is None:
            return target_speed
        if ramp_time <= 0 or step_interval <= 0:
            pwm_dev.value = max(PWM_DUTY_MIN, min(PWM_DUTY_MAX, target_speed / PWM_PERCENT_MAX))
            return target_speed
        steps = max(1, int(ramp_time / step_interval))
        step_duration = ramp_time / steps
        for step in range(1, steps + 1):
            if self._shutdown_active():
                pwm_dev.value = 0
                return 0.0
            duty = start_speed + (target_speed - start_speed) * (step / steps)
            pwm_dev.value = max(PWM_DUTY_MIN, min(PWM_DUTY_MAX, duty / PWM_PERCENT_MAX))
            time.sleep(step_duration)
        return target_speed

    def _ramp_pwm_dual(
        self,
        pwm_a,
        start_a,
        target_a,
        pwm_b,
        start_b,
        target_b,
        ramp_time,
        step_interval=MOTOR_RAMP_STEP,
    ):
        if self._shutdown_active():
            if pwm_a is not None:
                pwm_a.value = 0
            if pwm_b is not None:
                pwm_b.value = 0
            return 0.0, 0.0
        if pwm_a is None and pwm_b is None:
            return start_a, start_b
        if ramp_time <= 0 or step_interval <= 0:
            if pwm_a is not None:
                pwm_a.value = max(PWM_DUTY_MIN, min(PWM_DUTY_MAX, target_a / PWM_PERCENT_MAX))
            if pwm_b is not None:
                pwm_b.value = max(PWM_DUTY_MIN, min(PWM_DUTY_MAX, target_b / PWM_PERCENT_MAX))
            return target_a, target_b
        steps = max(1, int(ramp_time / step_interval))
        step_duration = ramp_time / steps
        for step in range(1, steps + 1):
            if self._shutdown_active():
                if pwm_a is not None:
                    pwm_a.value = 0
                if pwm_b is not None:
                    pwm_b.value = 0
                return 0.0, 0.0
            duty_a = start_a + (target_a - start_a) * (step / steps)
            duty_b = start_b + (target_b - start_b) * (step / steps)
            if pwm_a is not None:
                pwm_a.value = max(PWM_DUTY_MIN, min(PWM_DUTY_MAX, duty_a / PWM_PERCENT_MAX))
            if pwm_b is not None:
                pwm_b.value = max(PWM_DUTY_MIN, min(PWM_DUTY_MAX, duty_b / PWM_PERCENT_MAX))
            time.sleep(step_duration)
        return target_a, target_b

    def set_motor(
        self,
        motor_pwm,
        motor_dir,
        speed,
        forward,
        invert=False,
        ramp_time=MOTOR_RAMP_TIME,
        step_interval=MOTOR_RAMP_STEP,
    ):
        if self._shutdown_active():
            self.stop_motors()
            return
        if motor_pwm is None or motor_dir is None:
            return
        state = self.motor_state.setdefault(motor_pwm, {"speed": 0.0, "direction": True})
        current_speed = state["speed"]
        current_direction = state["direction"]

        if current_speed > 0 and forward != current_direction:
            current_speed = self._ramp_pwm(
                motor_pwm,
                current_speed,
                0.0,
                ramp_time / RAMP_HALF_DIVISOR,
                step_interval,
            )

        motor_dir.value = 1 if (forward ^ invert) else 0
        target_speed = self._clamp_percent(speed)
        devices = getattr(self, "devices", {})
        if motor_pwm is devices.get(DEVICE_MOTOR_1_PWM):
            target_speed = self._apply_motor_speed_scale(target_speed, 1)
        elif motor_pwm is devices.get(DEVICE_MOTOR_2_PWM):
            target_speed = self._apply_motor_speed_scale(target_speed, 2)
        current_speed = self._ramp_pwm(motor_pwm, current_speed, target_speed, ramp_time, step_interval)
        state["speed"] = current_speed
        state["direction"] = forward

    def set_motors(
        self,
        speed_a,
        forward_a,
        speed_b,
        forward_b,
        ramp_time=MOTOR_RAMP_TIME,
        step_interval=MOTOR_RAMP_STEP,
        cmd_type="set_motors",
    ):
        if self._shutdown_active():
            self.stop_motors()
            return
        # Fixed motor mapping:
        # - A => MTR1 => left
        # - B => MTR2 => right
        motor_1_pwm = self.devices.get(DEVICE_MOTOR_1_PWM)
        motor_1_dir = self.devices.get(DEVICE_MOTOR_1_DIR)
        motor_2_pwm = self.devices.get(DEVICE_MOTOR_2_PWM)
        motor_2_dir = self.devices.get(DEVICE_MOTOR_2_DIR)
        if motor_1_pwm is None or motor_1_dir is None or motor_2_pwm is None or motor_2_dir is None:
            return

        state_a = self.motor_state.setdefault(motor_1_pwm, {"speed": 0.0, "direction": True})
        state_b = self.motor_state.setdefault(motor_2_pwm, {"speed": 0.0, "direction": True})
        current_a = state_a["speed"]
        current_b = state_b["speed"]

        if (current_a > 0 and forward_a != state_a["direction"]) or (current_b > 0 and forward_b != state_b["direction"]):
            current_a, current_b = self._ramp_pwm_dual(
                motor_1_pwm,
                current_a,
                0.0,
                motor_2_pwm,
                current_b,
                0.0,
                ramp_time / RAMP_HALF_DIVISOR,
                step_interval,
            )

        motor_1_dir.value = 1 if (forward_a ^ MOTOR_DIR_INVERT_1) else 0
        motor_2_dir.value = 1 if (forward_b ^ MOTOR_DIR_INVERT_2) else 0

        target_a = self._apply_motor_speed_scale(speed_a, 1)
        target_b = self._apply_motor_speed_scale(speed_b, 2)
        current_a, current_b = self._ramp_pwm_dual(
            motor_1_pwm,
            current_a,
            target_a,
            motor_2_pwm,
            current_b,
            target_b,
            ramp_time,
            step_interval,
        )

        state_a["speed"] = current_a
        state_a["direction"] = forward_a
        state_b["speed"] = current_b
        state_b["direction"] = forward_b
        self._record_motor_command(cmd_type, current_a, forward_a, current_b, forward_b)

    def stop_motors(self):
        motor_1_pwm = self.devices.get(DEVICE_MOTOR_1_PWM)
        motor_2_pwm = self.devices.get(DEVICE_MOTOR_2_PWM)
        if motor_1_pwm:
            motor_1_pwm.value = 0
        if motor_2_pwm:
            motor_2_pwm.value = 0
        for state in self.motor_state.values():
            state["speed"] = 0.0
        self._record_motor_command("stop", 0.0, True, 0.0, True)
