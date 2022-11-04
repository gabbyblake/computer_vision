# import serial
import asyncio
from sentry import Sentry
from cmd_listener import SerialParser
from actuators import Display

# from motor_test import main

s = Sentry()
ser = SerialParser()

async def run_serial_test(ser:SerialParser, display:Display):
    while True:
        ser.test_serial_echo(display)
        await asyncio.sleep(0)



async def sentry_loop():
    led_task = asyncio.create_task(s.blink_led(0.08))


    # op_control_task = s.run_op_control(ser)
    # cmd_exection_task = s.execute_cmds()
    # await asyncio.gather(led_task, op_control_task, cmd_exection_task)  # Don't forget "await"!

    test_serial_task = run_serial_test(ser, s.display)
    await asyncio.gather(led_task, test_serial_task)

async def main():
    loop = asyncio.get_event_loop()
    loop.run_until_complete(sentry_loop())


try:
    asyncio.run(main())
except:
    print("steppers stopped")
    s.stepper_hold.toggle()
    while True:
        pass