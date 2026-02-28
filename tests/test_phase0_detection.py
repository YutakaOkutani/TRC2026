import time
import unittest

from cansat_mission.constants import Phase
from cansat_mission.phases.phase0 import Phase0Handler


class _FakeLed:
    def off(self):
        return None


class _FakeState:
    def __init__(self):
        self.phase = int(Phase.PHASE0)

    def update_navigation(self, phase=None, **_kwargs):
        if phase is not None:
            self.phase = phase


class _FakeController:
    def __init__(self):
        self.devices = {"led_red": _FakeLed(), "led_green": _FakeLed()}
        self.led_blink_timer = 0
        self.phase_entry_time = time.time()
        self.phase0_entry_marker = None
        self.phase0_initial_alt = None
        self.phase0_wait_log_counter = 0
        self.time_phase1_start = None
        self.bmp_last_valid_time = 0.0
        self.bmp_stale_sec = 99.0
        self.bno_last_acc_time = 0.0
        self.bno_acc_stale_sec = 99.0
        self.state = _FakeState()

    def toggle_led(self, *_args, **_kwargs):
        return None


class Phase0DetectionTest(unittest.TestCase):
    def setUp(self):
        self.handler = Phase0Handler()
        self.controller = _FakeController()

    def test_ignores_unavailable_sensor_defaults(self):
        snapshot = {"alt": 0.0, "fall": 0.0}

        self.handler.execute(self.controller, snapshot)

        self.assertEqual(self.controller.state.phase, int(Phase.PHASE0))
        self.assertIsNone(self.controller.phase0_initial_alt)

    def test_latches_initial_altitude_only_after_valid_bmp(self):
        snapshot = {"alt": 123.4, "fall": 0.0}
        self.controller.bmp_last_valid_time = time.time()
        self.controller.bmp_stale_sec = 0.0

        self.handler.execute(self.controller, snapshot)

        self.assertEqual(self.controller.phase0_initial_alt, 123.4)
        self.assertEqual(self.controller.state.phase, int(Phase.PHASE0))

    def test_transitions_on_altitude_drop_with_valid_bmp(self):
        self.controller.bmp_last_valid_time = time.time()
        self.controller.bmp_stale_sec = 0.0
        self.controller.phase0_initial_alt = 120.0
        self.controller.phase0_entry_marker = self.controller.phase_entry_time

        self.handler.execute(self.controller, {"alt": 40.0, "fall": 0.0})

        self.assertEqual(self.controller.state.phase, int(Phase.PHASE1))
        self.assertIsNotNone(self.controller.time_phase1_start)

    def test_transitions_on_fall_magnitude_with_valid_acc(self):
        self.controller.bno_last_acc_time = time.time()
        self.controller.bno_acc_stale_sec = 0.0

        self.handler.execute(self.controller, {"alt": 0.0, "fall": 35.0})

        self.assertEqual(self.controller.state.phase, int(Phase.PHASE1))
        self.assertIsNotNone(self.controller.time_phase1_start)


if __name__ == "__main__":
    unittest.main()
