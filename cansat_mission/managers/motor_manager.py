import time

from cansat_mission.constants import (
    APPROACH_TURN_GAIN,
    BASE_SPEED,
    CONE_CENTER_POSITION,
    DEVICE_MOTOR_1_DIR,
    DEVICE_MOTOR_1_PWM,
    DEVICE_MOTOR_2_DIR,
    DEVICE_MOTOR_2_PWM,
    GPS_TURN_CLAMP,
    GPS_TURN_GAIN,
    MOTOR_DIR_INVERT_1,
    MOTOR_DIR_INVERT_2,
    MOTOR_IDLE_SLEEP,
    MOTOR_LOOP_INTERVAL,
    MOTOR_RAMP_STEP,
    MOTOR_RAMP_TIME,
    OBSTACLE_AVOID_DIST,
    OBSTACLE_BACKUP_TIME,
    OBSTACLE_CONFIRM_COUNT,
    OBSTACLE_PAUSE_TIME,
    OBSTACLE_SPEED,
    OBSTACLE_TURN_TIME,
    PARACHUTE_DIRECTION,
    PARACHUTE_MOTOR_PULSE,
    PARACHUTE_SEPARATION_SPEED,
    PHASE2_SPEED,
    PHASE2_STAGE_STRAIGHT,
    PHASE2_TURN_BIAS,
    PHASE2_TURN_INTERVAL,
    PHASE3_NO_HEADING_SPEED,
    PHASE3_NO_HEADING_TURN_BIAS,
    PHASE3_NO_HEADING_TURN_INTERVAL,
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


class MotorManager:
    def _clamp_percent(self, value):
        return max(PWM_PERCENT_MIN, min(PWM_PERCENT_MAX, value))

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
        while True:
            snapshot = self.state.snapshot()
            phase = Phase(snapshot["phase"])
            obstacle_dist = snapshot["obstacle_dist"]
            direction = snapshot["direction"]
            cone_direction = snapshot["cone_direction"]

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
                self.set_motors(OBSTACLE_SPEED, False, OBSTACLE_SPEED, False, cmd_type="obstacle_backup")
                time.sleep(OBSTACLE_BACKUP_TIME)
                self.set_motors(OBSTACLE_SPEED, False, OBSTACLE_SPEED, True, cmd_type="obstacle_turn")
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
                    self.set_motors(PHASE2_SPEED, True, PHASE2_SPEED, True, cmd_type="phase2_straight")
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
                    self.set_motors(speed_r, True, speed_l, True, cmd_type="phase2_fig8")
                time.sleep(MOTOR_LOOP_INTERVAL)
                continue

            if phase == Phase.PHASE3:
                target_heading = direction
                fused_heading, _, total_weight = self._weighted_heading(snapshot)
                if fused_heading is not None:
                    self.phase3_no_heading_start = None
                    diff = self._angle_diff_deg(target_heading, fused_heading)
                    gain_scale = max(TURN_GAIN_SCALE_MIN, min(TURN_GAIN_SCALE_MAX, total_weight))
                    turn_val = diff * GPS_TURN_GAIN * gain_scale
                    turn_val = max(-GPS_TURN_CLAMP, min(GPS_TURN_CLAMP, turn_val))
                    speed_l = self._clamp_percent(BASE_SPEED + turn_val)
                    speed_r = self._clamp_percent(BASE_SPEED - turn_val)
                    self.set_motors(speed_r, True, speed_l, True, cmd_type="phase3_heading_follow")
                else:
                    if self.phase3_no_heading_start is None:
                        self.phase3_no_heading_start = time.time()
                    elapsed = time.time() - self.phase3_no_heading_start
                    left_turn = int(elapsed // PHASE3_NO_HEADING_TURN_INTERVAL) % 2 == 0
                    base = self._clamp_percent(PHASE3_NO_HEADING_SPEED)
                    bias = self._clamp_percent(PHASE3_NO_HEADING_TURN_BIAS)
                    if left_turn:
                        speed_l = self._clamp_percent(base - bias)
                        speed_r = self._clamp_percent(base + bias)
                    else:
                        speed_l = self._clamp_percent(base + bias)
                        speed_r = self._clamp_percent(base - bias)
                    self.set_motors(speed_r, True, speed_l, True, cmd_type="phase3_no_heading_search")
            elif phase == Phase.PHASE4:
                # Phase4でもカメラ正面姿勢を保ちつつ、既存の目標方位(direction)への誘導を維持する
                target_heading = direction
                fused_heading, _, total_weight = self._weighted_heading(snapshot)
                if fused_heading is not None:
                    self.phase3_no_heading_start = None
                    diff = self._angle_diff_deg(target_heading, fused_heading)
                    gain_scale = max(TURN_GAIN_SCALE_MIN, min(TURN_GAIN_SCALE_MAX, total_weight))
                    turn_val = diff * GPS_TURN_GAIN * gain_scale
                    turn_val = max(-GPS_TURN_CLAMP, min(GPS_TURN_CLAMP, turn_val))
                    speed_l = self._clamp_percent(BASE_SPEED + turn_val)
                    speed_r = self._clamp_percent(BASE_SPEED - turn_val)
                    self.set_motors(speed_r, True, speed_l, True, cmd_type="phase4_heading_follow")
                else:
                    # 方位が取れない時だけ探索回頭にフォールバック
                    self.set_motors(SEARCH_ROTATION_SPEED, True, SEARCH_ROTATION_SPEED, False, cmd_type="phase4_search_fallback")
            elif phase == Phase.PHASE5:
                err = cone_direction - CONE_CENTER_POSITION
                turn_cam = err * APPROACH_TURN_GAIN
                speed_l = self._clamp_percent(BASE_SPEED + turn_cam)
                speed_r = self._clamp_percent(BASE_SPEED - turn_cam)
                self.set_motors(speed_r, True, speed_l, True, cmd_type="phase5_approach")

            time.sleep(MOTOR_LOOP_INTERVAL)

    def _ramp_pwm(self, pwm_dev, start_speed, target_speed, ramp_time, step_interval=MOTOR_RAMP_STEP):
        if pwm_dev is None:
            return target_speed
        if ramp_time <= 0 or step_interval <= 0:
            pwm_dev.value = max(PWM_DUTY_MIN, min(PWM_DUTY_MAX, target_speed / PWM_PERCENT_MAX))
            return target_speed
        steps = max(1, int(ramp_time / step_interval))
        step_duration = ramp_time / steps
        for step in range(1, steps + 1):
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

        target_a = self._clamp_percent(speed_a)
        target_b = self._clamp_percent(speed_b)
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
