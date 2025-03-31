from w1thermsensor import W1ThermSensor
from gpiozero import PWMOutputDevice
from time import sleep
import json

class FanController:
    def __init__(self, fan_pin=18, min_temp=20.0, max_temp=45.0):
        self.sensor = W1ThermSensor()
        self.fan = PWMOutputDevice(fan_pin)
        self.min_temp = min_temp
        self.max_temp = max_temp

    def map_temp_to_speed(self, temperature):
        """Map temperature to a fan speed between 0.0 and 1.0."""
        if temperature <= self.min_temp:
            return 0.0
        elif temperature >= self.max_temp:
            return 1.0
        else:
            # Linear mapping between min_temp and max_temp
            return (temperature - self.min_temp) / (self.max_temp - self.min_temp)

    def run(self, verbal=True, interval=5):
        """Continuously adjust fan speed based on temperature and output JSON."""
        try:
            while True:
                temperature = self.sensor.get_temperature()
                fan_speed = self.map_temp_to_speed(temperature)
                self.fan.value = fan_speed

                if verbal:
                    # Prepare JSON output. Fan speed is expressed as a percentage.
                    data = {
                        "temperature": round(temperature, 2),
                        "fan_speed": round(fan_speed * 100, 0)
                    }
                    print(json.dumps(data))
                sleep(interval)
        except KeyboardInterrupt:
            self.shutdown()

    def shutdown(self):
        """Gracefully stop the fan."""
        self.fan.value = 0.0
        self.fan.close()
        print(json.dumps({"message": "Fan stopped."}))

if __name__ == "__main__":
    cooler = FanController(fan_pin=18, min_temp=20, max_temp=45)
    cooler.run()