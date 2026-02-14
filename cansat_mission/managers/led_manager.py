import time

from cansat_mission.constants import (
    DEVICE_LED_GREEN,
    DEVICE_LED_RED,
    LED_SIGNAL_SLEEP,
)


class LedManager:
    def toggle_led(self, led, timer, interval):
        if led is None:
            return
        if (timer // interval) % 2 == 0:
            led.on()
        else:
            led.off()

    def signal_led(self, times):
        led_red = self.devices.get(DEVICE_LED_RED)
        led_green = self.devices.get(DEVICE_LED_GREEN)
        for _ in range(times):
            if led_red:
                led_red.on()
            if led_green:
                led_green.on()
            time.sleep(LED_SIGNAL_SLEEP)
            if led_red:
                led_red.off()
            if led_green:
                led_green.off()
            time.sleep(LED_SIGNAL_SLEEP)
