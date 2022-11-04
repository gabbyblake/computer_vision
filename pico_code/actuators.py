
from digitalio import DigitalInOut, Direction
import asyncio
import pwmio
from microcontroller import Pin
from adafruit_motor import servo
import math
import busio
import displayio
import terminalio
from adafruit_display_text import label
import adafruit_displayio_ssd1306


microsteps = {
    1: [0, 0, 0],
    1/2: [1,0, 0],
    1/4: [0,1, 0],
    1/8: [1,1, 0],
    1/16: [1,1, 1],
}


class Display:
    def __init__(self, sda:Pin, scl:Pin,
                 width:int, height:int, border:int,
                 invert:bool=False) -> None:
        """ setup OLED display

        Args:
            sda (Pin): SDA pin
            scl (Pin): SCL pin
            width (int): width of display in px
            height (int): height of display in px
            border (int): border around outside of display
            invert (bool, optional): display colors inverted. Defaults to False.
        """
        # free up all pins that may have previously been used for displays
        displayio.release_displays()

        # Use for I2C
        i2c = busio.I2C(scl=scl, sda=sda); display_bus = displayio.I2CDisplay(i2c, device_address=0x3C)

        # setup display object
        self.WIDTH = width; self.HEIGHT = height; self.BORDER = border
        self.display = adafruit_displayio_ssd1306.SSD1306(display_bus, width=self.WIDTH, height=self.HEIGHT)

        self.setup_canvas()

    def setup_canvas(self):
        # Make the display context
        self.splash = displayio.Group()
        self.display.show(self.splash)

        color_bitmap = displayio.Bitmap(self.WIDTH, self.HEIGHT, 1)
        color_palette = displayio.Palette(1)
        color_palette[0] = 0xFFFFFF  # White

        bg_sprite = displayio.TileGrid(color_bitmap, pixel_shader=color_palette, x=0, y=0)
        self.splash.append(bg_sprite)

        self.clear_canvas()
    
    def clear_canvas(self):
        # Draw a smaller inner rectangle
        inner_bitmap = displayio.Bitmap(self.WIDTH - self.BORDER * 2, self.HEIGHT - self.BORDER * 2, 1)
        inner_palette = displayio.Palette(1)
        inner_palette[0] = 0x000000  # Black
        inner_sprite = displayio.TileGrid(
            inner_bitmap, pixel_shader=inner_palette, x=self.BORDER, y=self.BORDER
        )
        self.splash.append(inner_sprite)

    def text(self, text):
        self.clear_canvas()
        # create text object
        text_area = label.Label(
            terminalio.FONT, text=text, color=0xFFFFFF, y=self.HEIGHT // 2 - 1
        )

        # place text in x center of screen
        text_width = text_area.width
        text_area.x = math.floor(self.WIDTH/2 - text_width/2)
        self.splash.append(text_area)

class A4988:
    def __init__(self, DIR:Pin, STEP:Pin, max_steps_per_second:int=2156):
        """
        This class represents an A4988 stepper motor driver.  It uses two output pins
        for direction and step control signals.

        Args:
            DIR (Pin): pin on board connected to A4988 DIR pin
            STEP (Pin): pin on board connected to A4988 STEP pin
            max_speed (int): max speed of 
        """
        # This class represents an A4988 stepper motor driver.  It uses two output pins
        # for direction and step control signals.
        self.max_steps_per_second = max_steps_per_second

        # setup pins
        self._dir  = DigitalInOut(DIR); self._dir.direction  = Direction.OUTPUT
        self._step = pwmio.PWMOut(STEP, variable_frequency=True)
        
    def set_speed_pwm(self, speed):
        """
        Set the driver to run its stepper motor indefinitely at a given speed

        Args:
            speed (float): ranges from -1 to +1. Fraction of the max speed to drive at
        """
        if speed == 0: # stop the driver and return if speed is 0
            self._step.duty_cycle = 0
            return
        else: # determine step PWM freqeuncy and set the driver in motion
            # flip direction of movement if necessary
            if speed < 0:
                self._dir.value = False
            else:
                self._dir.value = True
            
            self._step.duty_cycle = 65535 // 2 # set duty cycle to 1/2

            f = math.floor(self.max_steps_per_second * abs(speed))  # calculate pwm frequency as percentage of maximum
            self._step.frequency = f           # set pwm frequency of step pin

    def __enter__(self):
        return self

    def __exit__(self):
        """ Automatically deinitializes the hardware when exiting a context. """
        self._dir.deinit()
        self._step.deinit()
        self._dir  = None
        self._step = None

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

        # stepper controller
        self.driver = A4988(DIR=direction_pin, STEP=step_pin)
    
    def set_speed(self, speed):
        """
        Move the motor indefinitely at a given speed

        Args:
            speed (float): -1 to 1, multiple of max speed
        """
        self.driver.set_speed_pwm(speed)

class Trigger:
    def __init__(self, servo_pin, flywheel_pin):
        """
        Represents trigger puller mechanism of the NERF gun, driven by an MG996R hobby servo.

        Args:
            servo_pin (board.pin): pin on the Pico connected to the signal lead of the servo
        """
        # state parameters
        self.safety_on = False
        
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
        # if self.safety_on:
        #     await asyncio.sleep(0)
        #     return
        # spin up flywheels if necessary
        # if not self.FLYWHEEL_ON.value:
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