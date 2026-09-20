import time
import gpiod


LED1 = 17
LED2 = 27
LED3 = 22


LED_PINS = [LED1, LED2, LED3]

request = gpiod.request_lines(
    "/dev/gpiochip0",
    consumer="leds",
    config={
        pin: gpiod.LineSettings(
            direction=gpiod.line.Direction.OUTPUT
        )
        for pin in LED_PINS
    },
)

request.set_value(17, gpiod.line.Value.ACTIVE)
request.set_value(27, gpiod.line.Value.ACTIVE)
request.set_value(22, gpiod.line.Value.ACTIVE)

while True:
    time.sleep(1)
    print("LEDs ON") 
    request.set_value(17, gpiod.line.Value.ACTIVE)
    request.set_value(27, gpiod.line.Value.ACTIVE)
    request.set_value(22, gpiod.line.Value.ACTIVE)
    time.sleep(1)
    print("LEDs OFF") 
    request.set_value(17, gpiod.line.Value.INACTIVE)
    request.set_value(27, gpiod.line.Value.INACTIVE)
    request.set_value(22, gpiod.line.Value.INACTIVE)
