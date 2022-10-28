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
from std_msgs.msg import String, Float64


class ObjectFirer(Node):

    def __init__(self, image_topic):
        """ Initialize the object identifier """
        super().__init__('object_identifier')
        self.cv_image = None                        # the latest image from the camera
        self.bridge = CvBridge()                    # used to convert ROS messages to OpenCV

        self.cap = cv2.VideoCapture(0)
        if not self.cap.isOpened():
            raise IOError("Cannot open webcam")

        self.create_subscription(Image, image_topic, self.process_image, 10)

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