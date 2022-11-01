import rclpy
from threading import Thread
from rclpy.node import Node
import time
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

        
        self.create_subscription(Float64MultiArray, pan_speed, tilt_speed, 10)

        self.vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.centroid_pub = self.create_publisher(Point, 'centroid', 10)
        self.pan_tilt_pub = self.create_publisher(Float64MultiArray, 'pan_tilt', 10)

        ser = serial.Serial(
            port='/dev/ttyUSB1',
            baudrate=9600,
            parity=serial.PARITY_ODD,
            stopbits=serial.STOPBITS_TWO,
            bytesize=serial.SEVENBITS
        )
        thread = Thread(target=self.loop_wrapper)
        thread.start()
        

  

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