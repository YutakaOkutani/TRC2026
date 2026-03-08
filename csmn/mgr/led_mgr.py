import time

from csmn.const import (
    DEVICE_LED_GREEN,
    DEVICE_LED_RED,
    LED_SIGNAL_SLEEP,
    LED_TIMEOUT_ALERT_FAST_SLEEP,
    LED_TIMEOUT_ALERT_FLASH_COUNT,
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

    def signal_total_timeout_alert(self):
        led_red = self.devices.get(DEVICE_LED_RED)
        led_green = self.devices.get(DEVICE_LED_GREEN)
        for i in range(LED_TIMEOUT_ALERT_FLASH_COUNT):
            # 通常パターンと区別しやすいよう、赤緑を高速で逆相点滅させる
            red_on = (i % 2) == 0
            green_on = not red_on
            if led_red:
                led_red.on() if red_on else led_red.off()
            if led_green:
                led_green.on() if green_on else led_green.off()
            time.sleep(LED_TIMEOUT_ALERT_FAST_SLEEP)
        if led_red:
            led_red.off()
        if led_green:
            led_green.off()

    def signal_give_up(self):
        led_red = self.devices.get(DEVICE_LED_RED)
        led_green = self.devices.get(DEVICE_LED_GREEN)
        if led_red:
            led_red.on()
        if led_green:
            led_green.off()
