# Code to drive the Comp-Robo robotic sentry NERF gun turret
# Run on a Raspberry Pi Pico with Circuitpy installed

# load standard modules
import math
import asyncio
import board
from digitalio import DigitalInOut, Direction

# load custom modules
from actuators import Stepper, StepperHold, Trigger, Display, microsteps

class PanTiltCmd:
    """
    Class that represents a stepper motor command for the pan/tilt mount on the sentry
    """

    def __init__(self, channel, speed) -> None:
        """
        Args:
            channel (string): "pan" for pan stepper, "tilt" for tilt stepper
            speed (float): -1 to +1. Fraction of the max speed to drive the channel at.
        """
        self.channel = channel
        self.speed = speed
    
    def __repr__(self):
        return f"{self.channel} Command with speed {self.speed}"
    
    def __eq__(self, other) : 
        return self.channel == other.channel
    
    def __hash__(self):
        return hash(self.channel)


class SpinCmd:
    """
    Class that represents a flywheel spin command for the sentry
    """

    def __init__(self, state) -> None:
        """
        Args:
            state (bool): True for spin up, False for spin down
        """
        self.state = state
        self.channel = "spin"
    
    def __repr__(self):
        return f"Spin Command with state {self.state}"

    def __eq__(self, other) : 
        return self.channel == other.channel
    
    def __hash__(self):
        return hash(self.channel)


class SafetyCmd:
    """
    Class that represents a trigger safety command for the sentry
    """

    def __init__(self, state) -> None:
        """
        Args:
            state (bool): True for safe on (trigger lock), False for safe off
        """
        self.state = state
        self.channel = "safety"
    
    def __repr__(self):
        return f"Safety Command with state {self.state}"

    def __eq__(self, other) : 
        return self.channel == other.channel
    
    def __hash__(self):
        return hash(self.channel)


class FireCmd:
    """
    Class that represents a fire command for the sentry trigger.
    """

    def __init__(self, state) -> None:
        self.state = state
        self.channel = "trigger"

    def __repr__(self):
        return "Fire Command"

    def __eq__(self, other) : 
        return self.channel == other.channel
    
    def __hash__(self):
        return hash(self.channel)


class Sentry:
    """
    Object that represents all of the actuators in the NERF sentry turret
    """

    def __init__(self, step_res = [1, 1, 1]):
        """
        Initialize all control objects and move them to their correct positions
        """
        # define pins on the Raspberry Pi Pico
        self.MS1 = board.GP20
        self.MS2 = board.GP21
        self.MS3 = board.GP22
        self.PAN_STP = board.GP17
        self.PAN_DIR = board.GP16
        self.TILT_STP = board.GP13
        self.TILT_DIR = board.GP12
        self.HOLD = board.GP11
        self.TRIGGER_SERVO = board.GP15
        self.FLYWHEEL_RELAY = board.GP28
        self.SDA = board.GP26
        self.SCL = board.GP27

        # setup board led
        self.led = DigitalInOut(board.LED)
        self.led.direction = Direction.OUTPUT

        # setuo microstep config pins for the A4988 stepper driver
        self.ms1 = DigitalInOut(self.MS1)
        self.ms1.direction = Direction.OUTPUT  # ms1 (microstep config) pin
        self.ms2 = DigitalInOut(self.MS2)
        self.ms2.direction = Direction.OUTPUT  # ms2 (microstep config) pin
        self.ms3 = DigitalInOut(self.MS3)
        self.ms3.direction = Direction.OUTPUT  # ms3 (microstep config) pin

        # set microstep config
        self.set_step_res(microsteps[1/16])

        # setup motor control objects
        # stepper that moves pan  channel
        self.pan_stepper = Stepper(4950, self.PAN_STP,  self.PAN_DIR)
        # stepper that moves tilt channel
        self.tilt_stepper = Stepper(1694, self.TILT_STP, self.TILT_DIR)
        # object for enabling/disabling stepper hold mode
        self.stepper_hold = StepperHold(self.HOLD)
        # object for controlling flywheel and trigger pull
        self.sentry_trigger = Trigger(self.TRIGGER_SERVO, self.FLYWHEEL_RELAY)

        # setup display
        self.display = Display(self.SDA, self.SCL, width=128, height=32, border=0)

        # set to hold currently active commands and the tasks that are serving them
        self.cmds = set()
    
    def __del__(self):
        self.stepper_hold.toggle()

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

    async def blink_led(self, interval):
        """
        Infinitely blink the onboard led of the pico at a given interval

        Args:
            interval (float): blink interval in seconds
        """
        while True:
            self.led.value = True
            # time.sleep(interval)
            await asyncio.sleep(interval)
            self.led.value = False
            # time.sleep(interval)
            await asyncio.sleep(interval)
    
    async def execute_cmds(self):
        """
        Asynchronously execute all motor commands

        Args:
            cmd (Cmd message): the command to execute
        """

        # switch on command message type to execute different types of commands
        while True:
            await asyncio.sleep(0)
            for cmd in self.cmds:
                await asyncio.sleep(0)
                # print(cmd)
                if isinstance(cmd, PanTiltCmd):
                    channel = cmd.channel
                    speed = cmd.speed
                    stepper = self.pan_stepper if channel == "pan" else self.tilt_stepper
                    stepper.set_speed(speed)
                elif isinstance(cmd, SpinCmd):
                    # TODO
                    pass
                elif isinstance(cmd, SafetyCmd):
                    # TODO
                    pass
                elif isinstance(cmd, FireCmd):
                    if cmd.state == True:
                        print("FIRE")
                        asyncio.create_task(self.sentry_trigger.fire())
                        # await asyncio.gather(firetask)
                        # await asyncio.sleep(0)
                        # firetask.cancel()

    async def run_op_control(self, ser):
        """
        Handles getting commands from serial parser and passing them to the execution method

        Args:
            ser (SerialParser): serial parser for the command stream
        """
        while True:
            await asyncio.sleep(0)
            input_cmds = ser.get_op_control_cmds()
            if input_cmds:
                # print(input_cmds)
                # async sleep to hopefully make shit work?
                await asyncio.sleep(0)

                # update command set
                for cmd in input_cmds:
                    await asyncio.sleep(0)
                    self.cmds.discard(cmd)
                    self.cmds.add(cmd)

                print(self.cmds)
