from time import sleep
import RPi.GPIO as gpio

direction_pins = [5, 12, 27] # brown
pulse_pins      = [13, 16, 19] # white
cw_direction    = 0 
ccw_direction   = 1 

gpio.setmode(gpio.BCM)
for direction_pin in direction_pins:
    gpio.setup(direction_pin, gpio.OUT)
    gpio.output(direction_pin,cw_direction)
for pulse_pin in pulse_pins:
    gpio.setup(pulse_pin, gpio.OUT)

try:
    while True:
        for x in range(1600):
            for pin in pulse_pins:
                gpio.output(pin,gpio.HIGH)
            sleep(1e-5)

            for pin in pulse_pins:
                gpio.output(pin,gpio.LOW)
            sleep(1e-5)

except KeyboardInterrupt:
    gpio.cleanup()