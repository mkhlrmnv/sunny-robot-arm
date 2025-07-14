from time import sleep
import threading
from gpiozero import DigitalOutputDevice
from cooling import FanController

# GPIO pin assignments
direction_pins = [26, 4, 19]  # brown wires
pulse_pins     = [13, 27, 20] # white wires

cw_direction  = False
ccw_direction = True

# Setup gpiozero devices
direction_outputs = [DigitalOutputDevice(pin) for pin in direction_pins]
pulse_outputs     = [DigitalOutputDevice(pin) for pin in pulse_pins]

# Set initial direction (clockwise)
for direction in direction_outputs:
    direction.value = cw_direction

# Start the fan controller in a separate thread
cooler = FanController()
fan_thread = threading.Thread(target=cooler.run, daemon=True)
fan_thread.start()

try:
    while True:
        for _ in range(1600):
            for pulse in pulse_outputs:
                pulse.on()
            sleep(1e-5)
            for pulse in pulse_outputs:
                pulse.off()
            sleep(1e-5)

except KeyboardInterrupt:
    print("Interrupted. Cleaning up.")
    for pin in direction_outputs + pulse_outputs:
        pin.close()