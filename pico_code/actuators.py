
from digitalio import DigitalInOut, Direction
import asyncio
import math
import pwmio
from adafruit_motor import servo


class A4988:
    def __init__(self, DIR, STEP):
        """This class represents an A4988 stepper motor driver.  It uses two output pins
        for direction and step control signals."""

        self._dir  = DigitalInOut(DIR)
        self._step = DigitalInOut(STEP)

        self._dir.direction  = Direction.OUTPUT
        self._step.direction = Direction.OUTPUT

        self._dir.value = False
        self._step.value = False

    def step(self, forward=True):
        """Emit one step pulse, with an optional direction flag."""
        self._dir.value = forward

        # Create a short pulse on the step pin.  Note that CircuitPython is slow
        # enough that normal execution delay is sufficient without sleeping.
        self._step.value = True
        self._step.value = False

    async def move(self, steps, speed):
        """Move the stepper motor the signed number of steps forward or backward at the
        speed specified in steps per second. Runs asynchronously (non blocking).
        """

        self._dir.value = (steps >= 0)
        time_per_step = 1.0 / speed
        for count in range(abs(steps)):
            self._step.value = True
            self._step.value = False
            await asyncio.sleep(time_per_step)

    def deinit(self):
        """Manage resource release as part of object lifecycle."""
        self._dir.deinit()
        self._step.deinit()
        self._dir  = None
        self._step = None

    def __enter__(self):
        return self

    def __exit__(self):
        # Automatically deinitializes the hardware when exiting a context.
        self.deinit()

class Stepper:
    def __init__(self, steps_per_rev, step_pin, direction_pin):
        """
        Represents and controls turret stepper motor driven by an A4988 control board

        Args:
            steps_per_rev (int): number of steps to turn the axis 360 deg (PAN = 4950, TILT = 1694)
            step_pin (board.pin): Pico GPIO pin connected to A4988 step (STEP) pin
            direction_pin (board.pin): Pico GPIO pin connected to A4988 direction (DIR) pin
            hold_pin (board.pin): Pico GPIO pin connected to A4988 hold toggle (ENABLE) pin
        """
        # control params
        self.steps_per_rev = steps_per_rev
        self.speed = 0 # (rev/sec)
        self.current_steps = 0

        # stepper controller
        self.driver = A4988(DIR=direction_pin, STEP=step_pin)
    
    async def move(self, move_speed, move_steps):
        """
        Move the stepper at a given speed. Max 800 steps/sec

        Args:
            move_speed (int): speed in steps/sec
        """
        self.current_steps += move_steps
        move_task = asyncio.create_task(self.driver.move(steps=move_steps, speed=move_speed))
        await asyncio.gather(move_task)
    
    async def move_deg(self, move_speed, angle_deg):
        """
        Move the axis by an angle at a given speed

        Args:
            move_speed (int): speed in steps/sec
            angle_deg (float): angle to move by in degrees
        """
        num_steps = math.floor(angle_deg / 360 * self.steps_per_rev)
        move_task = asyncio.create_task(self.move(move_speed, num_steps))
        await asyncio.gather(move_task)


    async def move_to_deg(self, move_speed, angle_deg):
        """
        Move the axis to a given angle at a given speed.

        Args:
            move_speed (int): steps/sec
            angle_deg (float): target in degrees
        """
        current_deg = self.current_steps / self.steps_per_rev * 360
        delta_deg = angle_deg - current_deg
        move_task = asyncio.create_task(self.move_deg(move_speed, delta_deg))
        await asyncio.gather(move_task)

class Trigger:
    def __init__(self, servo_pin, flywheel_pin):
        """
        Represents trigger puller mechanism of the NERF gun, driven by an MG996R hobby servo.

        Args:
            servo_pin (board.pin): pin on the Pico connected to the signal lead of the servo
        """
        # state parameters
        self.safety_on = True
        
        # Servo control object
        pwm = pwmio.PWMOut(servo_pin, duty_cycle=2 ** 15, frequency=50)
        self.servo = servo.Servo(pwm)

        # Flywheel motor relay control
        self.FLYWHEEL_ON = DigitalInOut(flywheel_pin)
        self.FLYWHEEL_ON.direction = Direction.OUTPUT
        self.FLYWHEEL_ON.value = False
    
    def toggle_safety(self):
        """
        Toggle the boolean state of trigger safety
        """
        self.safety_on = not self.safety_on
    
    async def fire(self):
        """
        Spin up flywheel if necessary and pull trigger servo to shoot one NERF dart
        """
        if self.safety_on:
            return
        # spin up flywheels if necessary
        if not self.FLYWHEEL_ON.value:
            self.FLYWHEEL_ON.value = True
            await asyncio.sleep(1.5)

        # pull trigger servo (only if it is in a retracted state)
        # if self.servo.angle < 10:
        self.servo.angle = 0
        await asyncio.sleep(1)
        self.servo.angle = 180
        await asyncio.sleep(.25)

        # spin down flywheels
        self.FLYWHEEL_ON.value = False

class StepperHold:
    def __init__(self, hold_pin):
        """
        Controls the hold function of the A4988 stepper driver

        Args:
            hold_pin (board.pin): Pico pin connected to the hold (ENABLE) pin of the A4988
        """
        # hold pin
        self.DISABLE_HOLD = DigitalInOut(hold_pin)
        self.DISABLE_HOLD.direction = Direction.OUTPUT
        self.DISABLE_HOLD.value = False
    
    def toggle(self):
        """
        Hold on powers the stepper to resist backdriving, hold off lets it spin freely.
        """
        self.DISABLE_HOLD.value = not self.DISABLE_HOLD.value
