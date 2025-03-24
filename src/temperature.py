from w1thermsensor import W1ThermSensor

sensor = W1ThermSensor()

while True:
    temperature = sensor.get_temperature()
    print(f"Temperature: {temperature:.2f}°C")
