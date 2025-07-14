import requests
import time

class Lamp:
    def __init__(self,
                 brightness=255,
                 https_url='http://192.168.1.120/win'):
        
        assert 0 <= brightness <= 255, "Brightness must be between 0 and 255"
        assert isinstance(https_url, str), "URL must be a string"
        assert https_url.startswith('http://') or https_url.startswith('https://'), "URL must start with http:// or https://"
        
        self.brightness = brightness
        self.https_url = https_url

        self.red = 255
        self.green = 255
        self.blue = 255

        self.effect = 0  # 0: solid, 1: blink, etc.
        self.speed = 200


    def set_brightness(self, brightness, verbal=False):
        assert 0 <= brightness <= 255, "Brightness must be between 0 and 255"

        url = f"{self.https_url}&A={brightness}"
        response = requests.get(url)
        
        if response.status_code != 200:
            raise Exception(f"Failed to set color: {response.status_code} - {response.text}")
        
        self.brightness = brightness
        
        if verbal:
            print(f"Set brightness to {self.brightness}")


    def set_color(self, r, g, b, verbal=False):
        assert 0 <= r <= 255, "Red value must be between 0 and 255"
        assert 0 <= g <= 255, "Green value must be between 0 and 255"
        assert 0 <= b <= 255, "Blue value must be between 0 and 255"

        # Send the color to the lamp via HTTP
        url = f"{self.https_url}&A={self.brightness}&R={r}&G={g}&B={b}"
        response = requests.get(url)

        if response.status_code != 200:
            raise Exception(f"Failed to set color: {response.status_code} - {response.text}")
        
        self.red = r
        self.green = g
        self.blue = b
        
        if verbal:
            print(f"Set color to R={r}, G={g}, B={b} with brightness {self.brightness}")


    def set_to_effect(self, r=255, g=0, b=0, effect_index=0, speed=200, verbal=False):
        assert 0 <= r <= 255, "Red value must be between 0 and 255"
        assert 0 <= g <= 255, "Green value must be between 0 and 255"
        assert 0 <= b <= 255, "Blue value must be between 0 and 255"
        assert isinstance(effect_index, int), "Effect index must be an integer"
        assert isinstance(speed, int), "Speed must be an integer"

        url = f"{self.https_url}&A={self.brightness}&R={r}&G={g}&B={b}&FX={effect_index}&SX={speed}"
        response = requests.get(url)

        if response.status_code != 200:
            raise Exception(f"Failed to set blink: {response.status_code} - {response.text}")
        
        self.red = r
        self.green = g
        self.blue = b
        self.effect_index = effect_index
        
        if verbal:
            print(f"Set to blink with color R={r}, G={g}, B={b}, effect index {effect_index}, speed {speed} ms")


    def set_to_solid(self, r=255, g=255, b=255, verbal=False):
        self.set_to_effect(r, g, b, effect_index=0, speed=200, verbal=verbal)


    def set_to_blink(self, r=255, g=0, b=0, speed=200, verbal=False):
        self.set_to_effect(r, g, b, effect_index=1, speed=speed, verbal=verbal)


if __name__ == "__main__":
    lamp = Lamp()

    lamp.set_to_solid()  # Set color to red