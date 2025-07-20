"""
cooling.py

A simple temperature‑controlled fan script for the Raspberry Pi.

It uses:
* w1thermsensor.W1ThermSensor  – to read a 1‑Wire DS18B20 temperature sensor.
* gpiozero.PWMOutputDevice   – to drive a 3‑pin/4‑pin PWM fan.
The script maps the measured temperature linearly onto a PWM duty cycle,
keeping the fan off below *min_temp* and at full speed above *max_temp*.
"""

from w1thermsensor import W1ThermSensor         # Temperature sensor driver
from gpiozero import PWMOutputDevice            # Hardware‑PWM wrapper
from time import sleep                          # Simple delay helper
from config import *

# A mutable global that external processes/threads can poll
# to get the most recent measurement and fan setting.
latest_reading = {"temperature": "--", "fan_speed": "--"}

class FanController:
    """Encapsulates sensor reading and fan speed control logic.

    Parameters
    ----------
    fan_pin : int, default 18
        The BCM GPIO pin providing hardware‑PWM for the fan.
    min_temp : float, default 30.0
        Temperature (°C) below which the fan is kept completely off.
    max_temp : float, default 60.0
        Temperature (°C) above which the fan is forced to 100 % duty cycle.
    """

    def __init__(self, fan_pin: int = FAN_PIN, 
                 min_temp: float = MIN_TEMP_IN_BOX, 
                 max_temp: float = MAX_TEMP_IN_BOX):
        
        self.sensor = W1ThermSensor()           # DS18B20 on the default 1‑Wire bus
        self.fan = PWMOutputDevice(fan_pin)     # Hardware‑PWM device (0.0 – 1.0)
        self.min_temp = min_temp
        self.max_temp = max_temp
        self.latest_reading = None              # Instance copy; updated every cycle

    def map_temp_to_speed(self, temperature: float) -> float:
        """Translate *temperature* in °C to a PWM duty cycle (0.0 – 1.0)."""
        # Completely off
        if temperature <= self.min_temp:
            return 0.0
        
        # Flat‑out cooling
        elif temperature >= self.max_temp:
            return 1.0
        
        # Linear ramp between the thresholds
        return (temperature - self.min_temp) / (self.max_temp - self.min_temp)

    def run(self, verbal: bool = True, interval: float = 0.1) -> None:
        """Main control loop.

        Continuously:
        1. Read the temperature.
        2. Convert it to a duty cycle.
        3. Apply the value to the fan.
        4. Store the reading for external use.
        5. Sleep for *interval* seconds.

        Press *Ctrl‑C* to exit and call :py:meth:`shutdown`.
        """
        global latest_reading

        try:
            while True:
                temperature = self.sensor.get_temperature()

                fan_speed = self.map_temp_to_speed(temperature)
                self.fan.value = fan_speed      # gpiozero expects 0.0 – 1.0

                # Update both instance and module‑level read‑outs
                latest_reading = {
                    "temperature": round(temperature, 2),
                    "fan_speed": round(fan_speed * 100, 0)
                }

                if verbal:
                    print(f"Temperature: {temperature:.2f}°C -> Fan Speed: {fan_speed * 100:.0f}%")
                sleep(interval)

        except KeyboardInterrupt:
            # Allow a clean exit when the user hits Ctrl‑C.
            self.shutdown()

    def shutdown(self):
        """Stop the fan and release GPIO resources."""
        self.fan.value = 0.0        # Ensure the fan is off
        self.fan.close()            # Clean up the PWM pin
        print("Fan stopped.")

if __name__ == "__main__":
    # Tweak *min_temp* and *max_temp* to suit your hardware or environment.
    cooler = FanController()
    cooler.run()
