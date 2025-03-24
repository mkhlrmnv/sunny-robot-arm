from gpiozero import PWMOutputDevice
from time import sleep

fan = PWMOutputDevice(18)  # BCM GPIO17 = physical pin 11

try:
    while True:
        fan.value = 1.0
        print("Fan Speed: 100%")
        sleep(5)

        fan.value = 0.6
        print("Fan Speed: 60%")
        sleep(5)

        fan.value = 0.2
        print("Fan Speed: 20%")
        sleep(5)

        fan.value = 0.0
        print("Fan Speed: 0%")
        sleep(5)

except KeyboardInterrupt:
    fan.close()
    print("Fan stopped.")
