# load standard Python modules
import time
import asyncio
# load CircuitPython modules
import board
# load custom modules
from actuators import Trigger, StepperHold
from PicoMotorLib import A4988Nema
from sentry import Trigger

### STARTUP ###
# define motor control pins
STEP_RES       = (board.GP20, board.GP21, board.GP22)
PAN_STP        = board.GP17
PAN_DIR        = board.GP16
TILT_STP       = board.GP13
TILT_DIR       = board.GP12
HOLD           = board.GP11
TRIGGER_SERVO  = board.GP15
FLYWHEEL_RELAY = board.GP28

# define motor control objects
pan_stepper  = A4988Nema(direction_pin=PAN_DIR, step_pin=PAN_STP, mode_pins=STEP_RES)
tilt_stepper = A4988Nema(TILT_DIR, TILT_STP)
sentry_trigger = Trigger(TRIGGER_SERVO, FLYWHEEL_RELAY)
hold = StepperHold(HOLD)

tilt_stepper.resolution_set("1/16")
time.sleep(1)

async def main():
    # test pan stepper
    print("moving pan")
    
    pan_task = asyncio.create_task(pan_stepper.motor_go(steps=20, stepdelay=.05))
    await asyncio.gather(pan_task)  # Don't forget the await!
    
    hold.toggle()
    while 1:
        pan_stepper.motor_stop()

    # # test tilt stepper
    # tilt_task = asyncio.create_task(tilt_stepper.motor_go(steps=20))
    # await asyncio.gather(tilt_task)  # Don't forget the await!

    # # test trigger
    # sentry_trigger.toggle_safety()
    # print("safety off fire")
    # sentry_trigger.fire()

asyncio.run(main())