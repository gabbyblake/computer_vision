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

class ObjectIdentifier(Node):

    def __init__(self, image_topic):
        """ Initialize the object identifier """
        super().__init__('object_identifier')
        self.cv_image = None                        # the latest image from the camera
        self.bridge = CvBridge()                    # used to convert ROS messages to OpenCV

        self.cap = cv2.VideoCapture(0)
        if not self.cap.isOpened():
            raise IOError("Cannot open webcam")

        self.create_subscription(Image, image_topic, self.process_image, 10)
        self.vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.centroid_pub = self.create_publisher(Point, 'centroid', 10)
        self.pan_speed_pub = self.create_publisher(Float64, 'pan_speed', 10)
        self.tilt_speed_pub = self.create_publisher(Float64, 'tilt_speed', 10)

        # self.red_lower_bound = 0
        # self.green_lower_bound = 0
        # self.blue_lower_bound = 0
        # self.red_upper_bound = 255
        # self.green_upper_bound = 255
        # self.blue_upper_bound = 255
        thread = Thread(target=self.loop_wrapper)
        thread.start()
        

    def process_image(self, msg):
        """ Process image messages from ROS and stash them in an attribute
            called cv_image for subsequent processing """
        self.cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

    def loop_wrapper(self):
        """ This function takes care of calling the run_loop function repeatedly.
            We are using a separate thread to run the loop_wrapper to work around
            issues with single threaded executors in ROS2 """
        cv2.namedWindow('video_window')
        cv2.namedWindow('binary_window')
        cv2.namedWindow('image_info')
        # self.red_lower_bound = 0
        # cv2.createTrackbar('red lower bound', 'binary_window', self.red_lower_bound, 255, self.set_red_lower_bound)
        # cv2.createTrackbar('red upper bound', 'binary_window', self.red_upper_bound, 255, self.set_red_upper_bound)
        # cv2.createTrackbar('green lower bound', 'binary_window', self.green_lower_bound, 255, self.set_green_lower_bound)
        # cv2.createTrackbar('green upper bound', 'binary_window', self.green_upper_bound, 255, self.set_green_upper_bound)
        # cv2.createTrackbar('blue lower bound', 'binary_window', self.blue_lower_bound, 255, self.set_blue_lower_bound)
        # cv2.createTrackbar('blue upper bound', 'binary_window', self.blue_upper_bound, 255, self.set_blue_upper_bound)
        cv2.setMouseCallback('video_window', self.process_mouse_event)
        while True:
            self.run_loop()
            time.sleep(0.1)

    def set_red_lower_bound(self, val):
        """ A callback function to handle the OpenCV slider to select the red lower bound """
        self.red_lower_bound = val

    def set_red_upper_bound(self, val):
        """ A callback function to handle the OpenCV slider to select the red upper bound """
        self.red_upper_bound = val

    def set_green_lower_bound(self, val):
        """ A callback function to handle the OpenCV slider to select the green lower bound """
        self.green_lower_bound = val

    def set_green_upper_bound(self, val):
        """ A callback function to handle the OpenCV slider to select the green upper bound """
        self.green_upper_bound = val

    def set_blue_lower_bound(self, val):
        """ A callback function to handle the OpenCV slider to select the blue lower bound """
        self.blue_lower_bound = val

    def set_blue_upper_bound(self, val):
        """ A callback function to handle the OpenCV slider to select the blue upper bound """
        self.blue_upper_bound = val

    def process_mouse_event(self, event, x,y,flags,param):
        """ Process mouse events so that you can see the color values
            associated with a particular pixel in the camera images """
        self.image_info_window = 255*np.ones((500,500,3))
        cv2.putText(self.image_info_window,
                    'Color (b=%d,g=%d,r=%d)' % (self.cv_image[y,x,0], self.cv_image[y,x,1], self.cv_image[y,x,2]),
                    (5,50),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    1,
                    (0,0,0))

    def run_loop(self):
        # NOTE: only do cv2.imshow and cv2.waitKey in this function 
        ret, frame = self.cap.read()
        self.cv_image = cv2.resize(frame, None, fx=0.5, fy=0.5, interpolation=cv2.INTER_AREA)
        
        
        
        if not self.cv_image is None:
            self.binary_image = cv2.inRange(self.cv_image, (0,0,115), (137,61,255))
            # self.binary_image = cv2.inRange(self.cv_image, (self.blue_lower_bound,self.green_lower_bound,self.red_lower_bound), (self.blue_upper_bound,self.green_upper_bound,self.red_upper_bound))
            print(self.cv_image.shape)

            msg = self.find_centroids()
            self.centroid_pub.publish(msg)
            
            cv2.imshow('video_window', self.cv_image)
            cv2.imshow('binary_window', self.binary_image)
            if hasattr(self, 'image_info_window'):
                cv2.imshow('image_info', self.image_info_window)
            cv2.waitKey(5)
    
    
    def find_centroids(self):
        # calculate moments of binary image
            M = cv2.moments(self.binary_image)
            # calculate x,y coordinates of center
            if M["m00"] != 0:
                cX = int(M["m10"] / M["m00"])
                cY = int(M["m01"] / M["m00"])
            else:
                cX, cY = 0, 0

            
            # put text and highlight the center
            cv2.circle(self.cv_image, (cX, cY), 5, (0, 255, 255), -1)
            cv2.putText(self.cv_image, "centroid", (cX - 25, cY - 25),cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)
            
            msg = Point(cX, cY, 0.0)
            return msg

if __name__ == '__main__':
    node = ObjectIdentifier("/camera/image_raw")
    node.run()


def main(args=None):
    rclpy.init()
    n = ObjectIdentifier("camera/image_raw")
    rclpy.spin(n)
    rclpy.shutdown()


if __name__ == '__main__':
    main()