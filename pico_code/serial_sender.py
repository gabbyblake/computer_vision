import serial
import time
# ser = serial.Serial("/dev/tty.usbmodem142103")  # open first serial port


def send_line(ser_object, line):
    """
        Send a line to the pico via serial connection

        Args:
            line (string): 1-line message to send, no return character
        """
    ser_object.write(bytes(line + "\n", "utf_8"))      # write a string


with serial.Serial("/dev/tty.usbmodem142203") as ser:  # open serial port
    # test string to send
    test_str = "ligma2"
    send_line(ser, test_str)
    time.sleep(0.01)
    print(f"{test_str} sent")
