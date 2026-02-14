import time

from cansat_mission.constants import (
    BNO_CALIB_MAG_MIN,
    DEVICE_LED_GREEN,
    DEVICE_LED_RED,
    LED_INTERVAL_PHASE2,
    PHASE2_FIG8_TIME,
    PHASE2_STAGE_FIG8,
    PHASE2_STAGE_STRAIGHT,
    PHASE2_STRAIGHT_TIME,
    PHASE_LOG_INTERVAL,
    Phase,
    TIMEOUT_PHASE_2,
)
from cansat_mission.phases.base import BasePhaseHandler


class Phase2Handler(BasePhaseHandler):
    def execute(self, controller, snapshot):
        led_red = controller.devices.get(DEVICE_LED_RED)
        led_green = controller.devices.get(DEVICE_LED_GREEN)
        print("phase2 : BNO Phase2 Calibration (Straight + Figure-8)")
        if led_red:
            led_red.off()
        if led_green:
            led_green.on()
        if controller.phase2_start_time is None:
            controller.phase2_start_time = time.time()
            controller.phase2_stage = PHASE2_STAGE_STRAIGHT
            controller.phase2_stage_start = controller.phase2_start_time
        now = time.time()
        elapsed = now - controller.phase2_start_time
        controller.led_blink_timer += 1
        controller.toggle_led(led_red, controller.led_blink_timer, interval=LED_INTERVAL_PHASE2)

        if controller.phase2_stage == PHASE2_STAGE_STRAIGHT and now - controller.phase2_stage_start >= PHASE2_STRAIGHT_TIME:
            controller.phase2_stage = PHASE2_STAGE_FIG8
            controller.phase2_stage_start = now

        calib = controller.bno_calib
        calib_ok = calib["valid"] and calib["value"][3] >= BNO_CALIB_MAG_MIN
        if controller.led_blink_timer % PHASE_LOG_INTERVAL == 0 and calib["valid"]:
            sys_st, gyro_st, accel_st, mag_st = calib["value"]
            print(f"Calib Status: Sys={sys_st} Gyro={gyro_st} Acc={accel_st} Mag={mag_st}")

        if calib_ok:
            print("Calibration OK: Mag >= threshold")
            controller.state.update_navigation(phase=int(Phase.PHASE3))
            controller.time_phase3_start = now
            return
        if controller.phase2_stage == PHASE2_STAGE_FIG8 and now - controller.phase2_stage_start >= PHASE2_FIG8_TIME:
            print("Phase2: Figure-8 complete -> Phase3")
            controller.state.update_navigation(phase=int(Phase.PHASE3))
            controller.time_phase3_start = now
            return
        if elapsed > TIMEOUT_PHASE_2:
            print("Phase2 TIMEOUT: Force Phase 3 (Calibration Incomplete)")
            controller.state.update_navigation(phase=int(Phase.PHASE3))
            controller.time_phase3_start = now


def run_standalone():
    from cansat_mission.runners import run_single_phase

    run_single_phase(Phase.PHASE2)


if __name__ == "__main__":
    run_standalone()
