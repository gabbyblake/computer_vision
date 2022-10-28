from computer_vision.object_identifier import ObjectIdentifier
from computer_vision.object_tracker import ObjectTracker
from computer_vision.object_firer import ObjectFirer
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
from std_msgs.msg import Float64

class Sentry(Node):

    def __init__(self, image_topic):
        """ Initialize the sentry """
        super().__init__('sentry')
        
        self.identify_node = ObjectIdentifier()
        self.track_node = ObjectTracker()
        self.fire_node = ObjectFirer()
        
        # create publishers
        self.vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.centroid_pub = self.create_publisher(Point, 'centroid', 10)
        self.pan_speed_pub = self.create_publisher(Float64, 'pan_speed', 10)
        self.tilt_speed_pub = self.create_publisher(Float64, 'tilt_speed', 10)

        thread = Thread(target=self.loop_wrapper)
        thread.start()
        


    def loop_wrapper(self):
        """ This function takes care of calling the run_loop function repeatedly.
            We are using a separate thread to run the loop_wrapper to work around
            issues with single threaded executors in ROS2 """
        cv2.namedWindow('video_window')
        cv2.namedWindow('binary_window')
        cv2.namedWindow('image_info')

        while True:
            self.run_loop()
            time.sleep(0.1)


    def run_loop(self):
        self.identify_node.run_loop()
        
    
    

if __name__ == '__main__':
    node = Sentry("/camera/image_raw")
    node.run()


def main(args=None):
    rclpy.init()
    n = Sentry("camera/image_raw")
    rclpy.spin(n)
    rclpy.shutdown()


if __name__ == '__main__':
    main()