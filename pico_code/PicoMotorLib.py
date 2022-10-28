#!/usr/bin/env python3
"""A python 3 library for various
 motors and servos to connect to a raspberry pi"""
# ========================= HEADER ===================================
# title             :PicoMotorLib.py
# description       :A python 3 library for 2-coil NEMA stepper to connect to a raspberry pi pico
# This file is tested for a pico running circuitpython
# Bipolar Nema Stepper motor A4988  Driver = A4988Nema class
# Main author       :Ben Grant
# Adapted from Gavin Lyons's code at https://github.com/gavinlyonsrepo/PicoMotorLib

# ========================== IMPORTS ======================
# Import the system modules needed to run rpiMotorlib.py
import sys
import asyncio
from digitalio import DigitalInOut, Direction



class StopMotorInterrupt(Exception):
    """ Stop the motor """
    pass

class A4988Nema(object):
    """ Class to control a Nema bi-polar stepper motor with a A4988 also tested with DRV8825"""
    def __init__(self, direction_pin, step_pin, mode_pins, driver_type="A4988"):
        """ class init method 3 inputs
        (1) direction_pin type=board.pin , help=GPIO pin connected to DIR pin of driver
        (2) step_pin type=board.pin , help=GPIO pin connected to STEP of driver
        (3) mode_pins type=tuple of 3 board.pins, help=GPIO pins connected to
        Microstep Resolution pins MS1-MS3 of IC, can be set to (-1,-1,-1) to turn off
        GPIO resolution.
        (4) driver_type type=string, help=Type of motor driver ("A4988" or "DRV8825")
        """
        self.driver_type = driver_type

        # setup GPIO pins
        print(direction_pin)
        self.direction_pin = DigitalInOut(direction_pin)
        self.direction_pin.direction  = Direction.OUTPUT
        self.step_pin = DigitalInOut(step_pin)
        self.step_pin.direction  = Direction.OUTPUT
        if mode_pins[0] != -1:
            self.mode_pins = tuple([DigitalInOut(pin) for pin in mode_pins])
            for pin in self.mode_pins:
                pin.direction = Direction.OUTPUT
        else:
            self.mode_pins = False

        # release motor stop
        self.stop_motor = False

    def motor_stop(self):
        """ Stop the motor """
        self.stop_motor = True

    def resolution_set(self, steptype):
        """ method to calculate step resolution
        based on motor type and steptype"""
        if self.driver_type == "A4988":
            resolution = {'Full': (0, 0, 0),
                          'Half': (1, 0, 0),
                          '1/4': (0, 1, 0),
                          '1/8': (1, 1, 0),
                          '1/16': (1, 1, 1)}
        else:
            print("Error invalid driver_type: {}".format(self.driver_type))
            raise(ValueError)

        # error check stepmode
        if steptype in resolution:
            pass
        else:
            raise(ValueError(f"Error invalid steptype: '{steptype}'.\n Must be of:\n 'Full'\n 'Half'\n '1/4'\n '1/8'\n '1/16'"))

        if self.mode_pins != False:
            for idx, pin in enumerate(self.mode_pins):
                print(idx, self.mode_pins)
                pin.value = bool(resolution[steptype][idx])

    async def motor_go(self, clockwise=False, steptype="Full",
                 steps=200, stepdelay=.005, verbose=False, initdelay=.05):
        """ motor_go,  moves stepper motor based on 6 inputs

         (1) clockwise, type=bool default=False
         help="Turn stepper counterclockwise"
         (2) steptype, type=string , default=Full help= type of drive to
         step motor 5 options
            (Full, Half, 1/4, 1/8, 1/16) 1/32 for DRV8825 only
         (3) steps, type=int, default=200, help=Number of steps sequence's
         to execute. Default is one revolution , 200 in Full mode.
         (4) stepdelay, type=float, default=0.05, help=Time to wait
         (in seconds) between steps.
         (5) verbose, type=bool  type=bool default=False
         help="Write pin actions",
         (6) initdelay, type=float, default=1mS, help= Intial delay after
         GPIO pins initialized but before motor is moved.

        """
        self.stop_motor = False

        try:
            # dict resolution
            self.resolution_set(steptype)
            await asyncio.sleep(initdelay)

            for i in range(steps):
                if self.stop_motor:
                    raise StopMotorInterrupt
                else:
                    self.step_pin.value = True
                    await asyncio.sleep(stepdelay)
                    self.step_pin.value = False
                    await asyncio.sleep(stepdelay)
                    if verbose:
                        print("Steps count {}".format(i+1), end="\r", flush=True)

        except KeyboardInterrupt:
            print("User Keyboard Interrupt : PicoMotorLib:")
        except StopMotorInterrupt:
            print("Stop Motor Interrupt : PicoMotorLib: ")
        except Exception as motor_error:
            print(sys.exc_info()[0])
            print(motor_error)
            print("PicoMotorLib  : Unexpected error:")
        else:
            # print report status
            if verbose:
                print("\nPicoMotorLib, Motor Run finished, Details:.\n")
                print("Motor type = {}".format(self.driver_type))
                print("Clockwise = {}".format(clockwise))
                print("Step Type = {}".format(steptype))
                print("Number of steps = {}".format(steps))
                print("Step Delay = {}".format(stepdelay))
                print("Intial delay = {}".format(initdelay))
                print("Size of turn in degrees = {}"
                      .format(degree_calc(steps, steptype)))
        finally:
            # cleanup
            self.step_pin.value = False
            self.direction_pin.value = False
            if self.mode_pins != False:
                for idx, pin in enumerate(self.mode_pins):
                    pin.value = False

    def deinit(self):
        """Manage resource release as part of object lifecycle."""
        self.direction_pin.deinit()
        self.direction_pin  = None
        self.step_pin.deinit()
        self.step_pin = None

        for idx, pin in enumerate(self.mode_pins):
            pin.deinit()
            pin = None
    
    def __exit__(self):
        # Automatically deinitializes the hardware when exiting a context.
        self.deinit()


def degree_calc(steps, steptype):
    """ calculate and returns size of turn in degree
    , passed number of steps and steptype"""
    degree_value = {'Full': 1.8,
                    'Half': 0.9,
                    '1/4': .45,
                    '1/8': .225,
                    '1/16': 0.1125,
                    '1/32': 0.05625,
                    '1/64': 0.028125,
                    '1/128': 0.0140625}
    degree_value = (steps*degree_value[steptype])
    return degree_value
