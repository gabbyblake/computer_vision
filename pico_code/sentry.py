# Code to drive the Comp-Robo robotic sentry NERF gun turret
# Run on a Raspberry Pi Pico with Circuitpy installed

# load standard modules
import time
import asyncio
import board
from digitalio import DigitalInOut, Direction

# load custom modules
from actuators import Stepper, StepperHold, Trigger

class Sentry:
    """
    Object that represents all of the actuators in the NERF sentry turret
    """
    def __init__(self):
        """
        Initialize all control objects and move them to their correct positions
        """
        # define pins on the Raspberry Pi Pico
        self.MS1             = board.GP20
        self.MS2             = board.GP21
        self.MS3             = board.GP22
        self.PAN_STP         = board.GP17
        self.PAN_DIR         = board.GP16
        self.TILT_STP        = board.GP13
        self.TILT_DIR        = board.GP12
        self.HOLD            = board.GP11
        self.TRIGGER_SERVO   = board.GP15
        self.FLYWHEEL_RELAY  = board.GP28

        # setup board led
        self.led = DigitalInOut(board.LED)
        self.led.direction = Direction.OUTPUT

        # setuo microstep config pins for the A4988 stepper driver
        self.ms1 = DigitalInOut(self.MS1); self.ms1.direction = Direction.OUTPUT  # ms1 (microstep config) pin
        self.ms2 = DigitalInOut(self.MS2); self.ms2.direction = Direction.OUTPUT  # ms2 (microstep config) pin
        self.ms3 = DigitalInOut(self.MS3); self.ms3.direction = Direction.OUTPUT  # ms3 (microstep config) pin

        # setup motor control objects
        self.pan_stepper  = Stepper(4950, self.PAN_STP,  self.PAN_DIR)            # stepper that moves pan  axis
        self.tilt_stepper = Stepper(1694, self.TILT_STP, self.TILT_DIR)           # stepper that moves tilt axis
        self.stepper_hold = StepperHold(self.HOLD)                                # object for enabling/disabling stepper hold mode
        self.sentry_trigger = Trigger(self.TRIGGER_SERVO, self.FLYWHEEL_RELAY)    # object for controlling flywheel and trigger pull

        # put shit in its right place
        self.led.value = True # turn LED on

    def set_step_res(self, step_res_config):
        """
        Switch the step resolution of the stepper motors

        Args:
            step_res_config (tuple of bools): values to set the
            MS1, MS2, MS3 pins on the A4988 stepper driver board
        """
        self.ms1.value = step_res_config[0]
        self.ms2.value = step_res_config[1]
        self.ms3.value = step_res_config[2]

    async def motor_test_async(self):
        # test trigger
        self.sentry_trigger.toggle_safety()
        print("safety off: firing")
        trigger_task = asyncio.create_task(self.sentry_trigger.fire())

        # test steppers
        print("moving steppers")
        pan_task1 = asyncio.create_task(self.pan_stepper.move(500, 400))
        tilt_task1 = asyncio.create_task(self.tilt_stepper.move(500, 400))
        await asyncio.gather(pan_task1, tilt_task1)

        # pan_task2 = asyncio.create_task(pan_stepper.move(2000, -400))
        # tilt_task2 = asyncio.create_task(tilt_stepper.move(500, -400))
        # await asyncio.gather(pan_task2, tilt_task2)

        # finish test
        self.stepper_hold.toggle()
