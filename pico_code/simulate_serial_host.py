import serial
from serial.tools.list_ports import grep
from pynput import keyboard
import serial


def send_line(ser_object, line):
    """
        Send a line to the pico via serial connection

        Args:
            line (string): 1-line message to send, no return character
        """
    ser_object.write(bytes(line + "\n", "utf_8"))      # write a string

# Find serial port of Pico stream
possible_ports = ["/dev/tty.usbmodem142103",
                  "/dev/tty.usbmodem142203",
                  "/dev/tty.usbmodem142303"]
USB_port = None
for port_name in possible_ports:
    try:
        s = serial.Serial(port_name)
        s.close()
        USB_port = port_name
    except:
        pass

try:
    with serial.Serial(USB_port) as ser:  # open serial port
        pressed = set()

        def on_key_press(key):
            global pressed

            # only run if key is not already in set of pressed keys
            if not key in pressed:
                pressed.add(key)
                send_line(ser, f'{pressed}')

        def on_key_release(key):
            global pressed
            pressed.remove(key)
            if len(pressed) > 0:
                send_line(ser, f'{pressed}')
            else:
                send_line(ser, "")

        with keyboard.Listener(on_release=on_key_release, on_press=on_key_press) as listener:
            listener.join()
except:
    ValueError("No serial port could be found for Pico data stream")
