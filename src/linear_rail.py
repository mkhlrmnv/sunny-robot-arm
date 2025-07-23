"""
linear_rail.py
~~~~~~~~~~~~~~

GPIO‑zero based driver for a **lead‑screw linear stage** (a.k.a. linear rail).
The carriage is moved by a stepper motor through a gearbox and screw with
*lead* (a.k.a. *pitch*) defined in **millimetres per revolution**.

Features
--------
* Single‑step primitive with optional speed scaling (open‑loop; no accel).
* Relative / absolute positioning in **millimetres**.
* Simple *homing* routine using a normally‑closed **limit switch**.
* Supports multi‑process coordination via a shared ``multiprocessing.Namespace``
  (``shared.delta_r`` keeps track of current distance from home).

The code assumes the STEP/DIR driver counts *negative* steps when moving the
carriage *forward* (away from the home switch).  Adjust the sign comments if
your wiring differs.
"""

from gpiozero import DigitalOutputDevice
import time
from gpiozero import Button
from signal import pause

from config import *

from multiprocessing import Event, Manager


class LinearRail:
    """Lead‑screw stage controlled via STEP/DIR driver and limit switch.

    Parameters
    ----------
    shared : multiprocessing.Namespace
        Object providing a floating‑point attribute ``delta_r`` that stores the
        current carriage displacement [mm] – useful when several processes
        must share state.
    pulse_pin, dir_pin, limit_pin : int
        BCM GPIO numbers for STEP, DIR, and *limit switch* inputs.
    step_per_rev : int, default 1600
        Micro‑stepped pulses per motor shaft revolution.
    gear_ratio : float, default 1
        **Output‑shaft : motor‑shaft** reduction factor.  If the motor drives
        the screw directly use ``1``.
    pitch : float, default 10
        Screw lead in millimetres per *output‑shaft* revolution (after gearbox).
    min_delay, max_delay : float
        Range for pulse high/low time.  A *speed_percent* of **1** ⇒ *min_delay*
        (fastest), **0** ⇒ *max_delay* (slowest).
    """

    def __init__(self, 
                 shared, 
                 pulse_pin: int, 
                 dir_pin: int, 
                 limit_pin: int, 
                 step_per_rev: int = 1600, 
                 gear_ratio: float = RAIL_MOTOR_GEAR_RATIO, 
                 pitch: float = SCREW_DRIVE_PITCH, 
                 min_delay: float = MOTORS_MIN_DELAY, 
                 max_delay: float = MOTORS_MAX_DELAY) -> None:
        assert all(isinstance(p, int) for p in (pulse_pin, dir_pin, limit_pin)), (
            "GPIO pins must be integers"
        )
        assert step_per_rev > 0, "step_per_rev must be positive"
        assert gear_ratio > 0, "gear_ratio must be positive"
        assert pitch > 0, "pitch (mm/rev) must be positive"
        assert 0 <= min_delay <= max_delay, "0 <= min_delay <= max_delay required"
        
        self.shared = shared

        # GPIO
        self.pulse = DigitalOutputDevice(pulse_pin)
        self.direction = DigitalOutputDevice(dir_pin)

        # Limit switch (active‑low, bounce filtered)
        self.limit_event = Event()
        self.limit_switch = Button(limit_pin, pull_up=True, bounce_time=0.0001)
        self.limit_switch.hold_time = 0.5   # call *when_held* after 100 ms
        self.limit_switch.hold_repeat = True
        self.limit_switch.when_held = lambda: (print("Limit switch is pressed"), self.limit_event.set())
        self.limit_switch.when_released = lambda: (print("Limit switch is released"), self.limit_event.clear())
        
        # Mechanical parameters
        self.pitch: float = pitch  # Pitch of the linear rail in mm
        self.step_per_rev: int = step_per_rev
        self.gear_ratio: float = gear_ratio

        # Timing range
        self.min_delay: float = min_delay
        self.max_delay: float = max_delay
        
        # Runtime state 
        self.steps: int = 0
        self.angle: float = 0
        self.distance: float = 0

        self.stop = self.limit_event.is_set()  # Initialize stop state based on limit switch


    # Timing helper
    def calc_delay(self, speed_percent: float) -> float:
        """Linearly map *speed_percent* ∈ [0,1] → delay seconds."""
        if not (0 <= speed_percent <= 1):
            raise ValueError("Speed percent must be between 0 and 1.")
        
        return self.min_delay + (self.max_delay - self.min_delay) * (1 - speed_percent)

    # Homing routine
    def init_motor(self, direction: int = 1) -> None:
        """Seek the limit switch and back‑off ~90° to clear it."""
        while not self.limit_event.is_set():
            self.step(direction=direction, speed=0.1)

        # Back‑off 90° (output shaft) opposite to approach direction to release the limit switch
        self.move_by_distance(5, speed=0.5, ignore_limit=True)
        self.reset_position()
        print(f"Motor limit initialized, stop state {self.stop}")


    def step(self, 
             direction: int = 1, 
             speed: float = 0.5, 
             ignore_limit: bool = False) -> None:
        """Issue **one** STEP pulse and update counters."""

        if self.limit_event.is_set() and not ignore_limit:
            print("HARD LIMIT – move blocked")
            return

        if direction not in [-1, 1]:
            raise ValueError("Direction must be ±1")

        self.direction.value = 1 if direction == 1 else 0
        delay = self.calc_delay(speed)

        self.pulse.on(); time.sleep(delay)
        self.pulse.off(); time.sleep(delay)

        # Update logical state.
        self.steps += direction
        self.angle += direction * (360 / (self.step_per_rev * self.gear_ratio))
        self.angle = round(self.angle, 3)

        # Distance sign inverted to compensate wiring: dir=+1 → −distance.
        self.distance += (-1 * direction) * (self.pitch / self.step_per_rev)
        self.shared.delta_r += (-1 * direction) * (self.pitch / self.step_per_rev)


    def move_by_distance(self, 
                         distance: float, 
                         speed: float = 0.5,
                         ignore_limit: bool = False, 
                         ): #shared = None) -> None:
        """Translate carriage by *distance* millimetres (signed)."""

        if distance == 0:
            return

        steps_per_mm = self.step_per_rev / self.pitch  # after gearbox already accounted in *step_per_rev*
        steps = int(distance * steps_per_mm)
        direction = 1 if distance < 0 else -1 
        
        # because now init point is on the wrong side our motor works in wrong direction
        # so to get to use positive numbers when going right direction has to be flipped

        for _ in range(abs(steps)):
            self.step(direction=direction, speed=speed, ignore_limit=ignore_limit)

        # if shared is not None:    # <- TODO: in theory this can be deleted:)
        #     shared.delta_r = self.distance
    
    def move_to_distance(self, 
                         target_distance: float, 
                         speed: float = 0.5, 
                         ): # shared = None) -> None:
        """Translate until :pyattr:`distance` == *target_distance* mm."""
        if target_distance < 0:
            raise ValueError("Target distance must be non-negative.")
        
        self.distance = self.shared.delta_r
        
        distance_diff = target_distance - self.distance
        self.move_by_distance(distance_diff, speed=speed) # , shared=shared)


    def reset_position(self) -> None:
        self.steps = 0
        self.angle = 0.0
        self.distance = 0.0
        self.shared.delta_r = self.distance


    def cleanup(self) -> None:
        """Release GPIO resources."""
        self.pulse.close()
        self.direction.close()
        self.limit_switch.close()
        print("Motor linear resources cleaned up.")


if __name__ == "__main__":
    manager = Manager()
    shared = manager.Namespace()
    motor = LinearRail(pulse_pin=27, shared=shared, dir_pin=4, limit_pin=24, gear_ratio=1)
    # motor.move_by_angle(-720*6, speed=0.5)
    motor.init_motor(direction=1)
    # motor.move_by_distance(-500, speed=1)
    
    # motor.move_to_distance(10, speed=0.5)
    # print("moved")
    # print(motor.steps)
    # time.sleep(1)
    # motor.move_to_distance(40, speed=0.5)
    # print("moved")
    # print(motor.steps)
    # time.sleep(1)
    # motor.move_to_distance(30, speed=0.5)
    # print("moved")
    # print(motor.steps)
    # time.sleep(1)
    # motor.move_to_distance(50, speed=0.5)
    # print("moved")

