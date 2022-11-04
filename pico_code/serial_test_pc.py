# sender.py
import time
import serial

def main(args=None):
  ser = serial.Serial(
    port='/dev/ttyAMC0', # Change this according to connection methods, e.g. /dev/ttyUSB0
    baudrate = 115200,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE,
    bytesize=serial.EIGHTBITS,
    timeout=1
  )

  i = 0

  pan_tilt = [0.0, 0.5]
  pan_speed = pan_tilt[0]
  tilt_speed = pan_tilt[1]
  spin = "SPIN UP" # or "SPIN DOWN"
  safety = "SAFETY ON" # "or SAFETY OFF"
  fire = "FIRE"

  while True:
      i+=1
      print("Counter {} - Hello from Raspberry Pi".format(i))
      msg = "SET PAN " + str(pan_speed)
      ser.write(msg.encode('utf-8'))

      msg = "SET TILT " + str(tilt_speed)
      ser.write(msg.encode('utf-8'))

      msg = spin
      ser.write(msg.encode('utf-8'))

      msg = safety
      ser.write(msg.encode('utf-8'))

      msg = fire
      ser.write(msg.encode('utf-8'))
      time.sleep(2)




if __name__ == '__main__':
    main()