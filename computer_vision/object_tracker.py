from turtle import tilt
import rclpy
from threading import Thread
from rclpy.node import Node
import time
from sensor_msgs.msg import Image
from copy import deepcopy
from cv_bridge import CvBridge
import cv2
import numpy as np
from std_msgs.msg import float64
from geometry_msgs.msg import Twist, Vector3
from simple_pid import PID


class ObjectTracker(Node):

    def __init__(self, image_topic):
        """ Initialize the object tracker """
        super().__init__('object_tracker')
        self.cv_image = None                        # the latest image from the camera
        self.bridge = CvBridge()                    # used to convert ROS messages to OpenCV

        self.cap = cv2.VideoCapture(0)
        if not self.cap.isOpened():
            raise IOError("Cannot open webcam")

        self.create_subscription(list, centroid, self.process_centroid, 10)
        #self.motor_pub = self.create_publisher(list, 'stepper_command', 10)

        thread = Thread(target=self.loop_wrapper)
        thread.start()

        frame_width = self.cv_image.shape[1] # px
        frame_height = self.cv_image.shape[0] # px

        #PID VALUES TO BE TUNED
        self.pan_pid  = PID(Kp = 0.1, Ki = 0, Kd = 0.05, setpoint = (frame_width  / 2))    # PID controller for steering
        self.tilt_pid = PID(Kp = 0.5, Ki = 0, Kd = 0,    setpoint = (frame_height / 2))  # PID controller for linear velocity

    def process_centroid(self, msg):
        """ Input: Centroid Coordinates
            Output: Motor Commands """
            
        pan_cmd = float64()
        tilt_cmd = float64()
        centroid_x = msg[0]
        centroid_y = msg[1]

        pan_cmd = self.pan_pid(centroid_x)
        tilt_cmd = self.tilt_pid(centroid_y)

        self.motor_pub.publish(pan_cmd)
        self.motor_pub.publish(tilt_cmd)

if __name__ == '__main__':
    node = ObjectTracker("/camera/image_raw") # not sure if the input path should stay
    node.run()

def main(args=None):
    rclpy.init()
    n = ObjectTracker("camera/image_raw")
    rclpy.spin(n)
    rclpy.shutdown()


if __name__ == '__main__':
    main()