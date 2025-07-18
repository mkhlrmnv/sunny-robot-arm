"""
lamp.py

HTTP‑controlled RGB lamp helper for WLED or similar firmware.

This script wraps a small subset of the WLED JSON / API interface:
    * global brightness (0–255)
    * solid colour (R, G, B)
    * basic effects (index + speed)

It assumes the device accepts query‑parameter control as follows::

    http://<IP-or-host>/win&A=<brightness>&R=<red>&G=<green>&B=<blue>&FX=<effect>&SX=<speed>

Adjust those parameters if your firmware differs.

Example
-------
>>> lamp = Lamp(https_url="...")
>>> lamp.set_to_solid(r=255, g=255, b=0)     # yellow constant light
>>> lamp.set_to_blink(r=0, g=0, b=255, speed=100)  # fast blue strobe
"""

import requests
import time
import os
from dotenv import load_dotenv
load_dotenv() 

class Lamp:
    """Small convenience wrapper around a network‑attached RGB lamp.

    Parameters
    ----------
    brightness : int, default 255
        Global intensity (0 = off, 255 = maximum).
    https_url : str, default '...' TODO: <- add address here
        Base URL of the lamp's *win* control endpoint. **Must** include either
        ``http://`` or ``https://``.
    """

    def __init__(self,
                 brightness: int = 255,
                 https_url: str = os.getenv("WLED_ADR")) -> None:
        
        assert 0 <= brightness <= 255,          "Brightness must be between 0 and 255"
        assert isinstance(https_url, str),      "URL must be a string"
        assert https_url.startswith('http://') or https_url.startswith('https://'), "URL must start with http:// or https://"
        
        self.brightness: int = brightness
        self.https_url: str = https_url

        # Current RGB colour (stored client‑side for convenience only)
        self.red: int = 255
        self.green: int = 255
        self.blue: int = 255

        # Effect parameters (0 = solid), (1 = blink), ect. More info -> https://github.com/wled/WLED/wiki/List-of-effects-and-palettes
        self.effect: int = 0
        self.speed: int = 200    # effect speed in milliseconds


    def _send(self, query: str) -> None:
        """Helper that performs the GET request and raises on HTTP error."""
        url = f"{self.https_url}&{query}"
        print(url)
        response = requests.get(url, timeout=5)
        if response.status_code != 200:
            raise RuntimeError(
                f"Device returned {response.status_code}: {response.text[:120]}"
            )


    def set_brightness(self, brightness: int, verbal: bool = False) -> None:
        """Update the global brightness while keeping current colour."""
        assert 0 <= brightness <= 255, "Brightness must be between 0 and 255"

        self._send(f"A={brightness}")
        self.brightness = brightness
        
        if verbal:
            print(f"Set brightness to {self.brightness}")


    def set_color(self, r: int, g: int, b:int, verbal: bool = False) -> None:
        assert 0 <= r <= 255, "Red value must be between 0 and 255"
        assert 0 <= g <= 255, "Green value must be between 0 and 255"
        assert 0 <= b <= 255, "Blue value must be between 0 and 255"

        query = f"A={self.brightness}&R={r}&G={g}&B={b}"
        self._send(query)
        
        self.red, self.green, self.blue = r, g, b
        
        if verbal:
            print(f"Set color to R={r}, G={g}, B={b} with brightness {self.brightness}")


    def set_to_effect(self, 
                      r: int = 255, 
                      g: int = 0, 
                      b: int = 0, 
                      effect_index: int = 0, 
                      speed: int = 200, 
                      verbal: bool = False) -> None:
        
        """Generic effect setter.

        Parameters
        ----------
        r, g, b : int, default (255, 0, 0)
            Starting colour for the effect.
        effect_index : int, default 0
            The firmware‑defined effect ID (consult WLED docs) 
            -> https://github.com/wled/WLED/wiki/List-of-effects-and-palettes
        speed : int, default 200
            Effect speed in milliseconds.
        """

        assert 0 <= r <= 255, "Red value must be between 0 and 255"
        assert 0 <= g <= 255, "Green value must be between 0 and 255"
        assert 0 <= b <= 255, "Blue value must be between 0 and 255"
        assert isinstance(effect_index, int), "Effect index must be an integer"
        assert isinstance(speed, int), "Speed must be an integer"

        query = f"A={self.brightness}&R={r}&G={g}&B={b}&FX={effect_index}&SX={speed}"
        self._send(query)

        self.red, self.green, self.blue = r, g, b
        self.effect_index, self.speed = effect_index, speed
        
        if verbal:
            print(f"Set to blink with color R={r}, G={g}, B={b}, effect index {effect_index}, speed {speed} ms")


    def set_to_solid(self, 
                     r: int = 255, 
                     g: int = 255, 
                     b: int = 255, 
                     verbal: bool = False) -> None:
        self.set_to_effect(r, g, b, effect_index=0, speed=200, verbal=verbal)


    def set_to_blink(self, 
                     r: int = 255, 
                     g: int = 0, 
                     b: int = 0, 
                     speed: int = 200, 
                     verbal: bool = False) -> None:
        self.set_to_effect(r, g, b, effect_index=1, speed=speed, verbal=verbal)


if __name__ == "__main__":
    lamp = Lamp()
    
    # Demonstration: cycle through three modes
    try:
        lamp.set_to_solid(255, 0, 0, verbal=True)   # solid red
        time.sleep(2)

        lamp.set_to_solid(0, 255, 0, verbal=True)   # solid green
        time.sleep(2)

        lamp.set_to_blink(0, 0, 255, speed=100, verbal=True)  # blue strobe
        time.sleep(4)

    finally:
        # Graceful fallback to white at low brightness
        lamp.set_brightness(32)
        lamp.set_to_solid(255, 255, 255, verbal=True)
        