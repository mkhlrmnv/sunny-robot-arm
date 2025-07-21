"""
warning_sound.py
~~~~~~~~~~~~~~~~~~~~~~~~

A simple helper to play a looping warning sound in a background thread,
with start/stop control, using pygame for audio playback.

Classes
-------
WarningSound
    Control the playback of an audio file on infinite loop.

Usage
-----
>>> ws = WarningSound("alarm.wav")
>>> ws.start()  # begins playing sound in background
>>> ws.stop()   # stops playback cleanly
"""
import pygame
import threading
import time


class WarningSound:
    """
    A warning sound player that loops an audio file indefinitely in a background thread.

    Methods
    -------
    start()
        Begin playback (no-op if already playing).
    stop()
        Signal playback to stop and wait for thread to exit.
    """

    def __init__(self, audio_file: str) -> None:
        """
        Initialize the pygame mixer and load the specified audio file.

        Parameters
        ----------
        audio_file : str
            Path to the sound file to be looped (e.g., WAV or MP3).
        """
        # Initialize the mixer; must be called before loading/playing sounds
        pygame.mixer.init()
        # Load the music track for playback
        pygame.mixer.music.load(audio_file)
        # Threading event to signal when playback should stop
        self._stop = threading.Event()
        # Placeholder for the background thread
        self._thread = None

    def _play_loop(self) -> None:
        """
        Internal method run in a background thread.
        Plays the loaded audio file on infinite loop until _stop is set.
        """
        # Start playback with loops=-1 (pygame convention for infinite loop)
        pygame.mixer.music.play(loops=-1)
        # Keep the thread alive, polling the stop event periodically
        while not self._stop.is_set():
            time.sleep(0.2)
        # Once stop is requested, halt playback
        pygame.mixer.music.stop()

    def start(self) -> None:
        """
        Start looping the warning sound.
        If already playing, this is a no-op.
        """
        # If a thread exists and is alive, do nothing
        if self._thread and self._thread.is_alive():
            return
        # Clear any previous stop signal
        self._stop.clear()
        # Launch the playback loop in a daemon thread
        self._thread = threading.Thread(target=self._play_loop, daemon=True)
        self._thread.start()

    def stop(self) -> None:
        """
        Stop the warning sound and wait for the playback thread to finish.
        """
        # Signal the _play_loop to exit its loop
        self._stop.set()
        # If a thread was running, join it to clean up
        if self._thread:
            self._thread.join()
            self._thread = None


# -----------------------------------------------------------------------------
# Stand-alone execution for testing
# -----------------------------------------------------------------------------
if __name__ == "__main__":
    # Example: adjust path to your sound file
    ws = WarningSound("sounds/timanttei_leikattu.wav")
    # Begin playing the warning sound
    ws.start()
    print("Warning sound started. Press Ctrl+C to exit.")
    try:
        # Keep the main thread alive indefinitely
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("Stopping...")
        # On Ctrl+C, stop playback cleanly
        ws.stop()
        print("Exited cleanly.")
