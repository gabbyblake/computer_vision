import rclpy
from threading import Thread
from rclpy.node import Node
import time
import serial
from sensor_msgs.msg import Image
from copy import deepcopy
from cv_bridge import CvBridge
import cv2
import numpy as np
from geometry_msgs.msg import Twist, Point
from std_msgs.msg import String, Float64, Float64MultiArray
import serial


class ObjectFirer(Node):


    def __init__(self, image_topic):
        """ Initialize the object firer """
        super().__init__('object_firer')
        self.ser = serial.Serial(
            port='/dev/ttyACM0', # Change this according to connection methods, e.g. /dev/ttyUSB0
            baudrate = 115200,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            bytesize=serial.EIGHTBITS,
            timeout=1
        )

        
        self.create_subscription(Float64MultiArray, "pan_tilt", self.send_msg, 10)
        self.create_subscription()

        thread = Thread(target=self.loop_wrapper)
        thread.start()

      
    def send_msg(self, msg):
        pan_speed = msg[0]
        tilt_speed = msg[1]

        pan_msg = "SET PAN " + str(pan_speed) + "\n"
        self.ser.write(pan_msg.encode('utf-8'))

        tilt_msg = "SET TILT " + str(tilt_speed) + "\n"
        self.ser.write(tilt_msg.encode('utf-8'))

        msg =  "SPIN" + "\n"
        self.ser.write(msg.encode('utf-8'))

        msg = "SAFETY" + "\n"
        self.ser.write(msg.encode('utf-8'))

        msg = "FIRE" + "\n"
        self.ser.write(msg.encode('utf-8'))

        time.sleep(2)


      
        

  

    def run_loop(self):
        # do something
        print("fire turret")
    
    


if __name__ == '__main__':
    node = ObjectFirer("/camera/image_raw")
    node.run()


def main(args=None):
    rclpy.init()
    n = ObjectFirer("camera/image_raw")
    rclpy.spin(n)
    rclpy.shutdown()


if __name__ == '__main__':
    main()