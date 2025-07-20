# warning_sound_pygame.py
import pygame
import threading
import time

class WarningSound:
    def __init__(self, audio_file):
        pygame.mixer.init()
        pygame.mixer.music.load(audio_file)
        self._stop = threading.Event()
        self._thread = None

    def _play_loop(self):
        # -1 means infinite loop
        pygame.mixer.music.play(loops=-1)
        # just wait until asked to stop
        while not self._stop.is_set():
            time.sleep(0.2)
        pygame.mixer.music.stop()

    def start(self):
        if self._thread and self._thread.is_alive():
            return
        self._stop.clear()
        self._thread = threading.Thread(target=self._play_loop, daemon=True)
        self._thread.start()

    def stop(self):
        self._stop.set()
        if self._thread:
            self._thread.join()
            self._thread = None


if __name__ == "__main__":
    ws = WarningSound("sounds/timanttei_leikattu.wav")   # or .mp3
    ws.start()
    print("Warning sound started. Ctrl+C to stop.")
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("Stopping…")
        ws.stop()
        print("Exited cleanly.")