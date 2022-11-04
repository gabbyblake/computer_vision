import rclpy
from turtle import tilt
from threading import Thread
from rclpy.node import Node
import time
import serial
import numpy as np
from copy import deepcopy
from cv_bridge import CvBridge
import cv2
from sensor_msgs.msg import Image
from std_msgs.msg import String, Float64, Float64MultiArray
from geometry_msgs.msg import Point, Twist
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

        # delete these publish statements once sentry code works
        self.vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.motor_pub = self.create_publisher(Float64MultiArray, 'pan_tilt', 10)

        # self.ser = serial.Serial(
        #     port='/dev/ttyACM0', # Change this according to connection methods, e.g. /dev/ttyUSB0
        #     baudrate = 115200,
        #     parity=serial.PARITY_NONE,
        #     stopbits=serial.STOPBITS_ONE,
        #     bytesize=serial.EIGHTBITS,
        #     timeout=1
        # )



        # self.red_lower_bound = 0
        # self.green_lower_bound = 0
        # self.blue_lower_bound = 0
        # self.red_upper_bound = 255
        # self.green_upper_bound = 255
        # self.blue_upper_bound = 255

        self.create_subscription(Image, image_topic, self.process_image, 10)

        
        self.aim_box_radius = 20
        self.lock_timer = 0.0
        
        # initial guess of frame dimensions
        self.frame_width = 320 # px
        self.frame_height = 240 # px

        self.centroid = [160, 120]
        self.aim = [160, 120]

       

        # PIDs for simulator were here

        self.start_timer= time.time()
        self.clock = 0.0

        thread = Thread(target=self.loop_wrapper)
        thread.start()

    def send_pan_and_tilt_msg(self, msg):
        pan_speed = msg[0]
        tilt_speed = msg[1]

        pan_msg = "SET PAN " + str(pan_speed) + "\n"
        self.ser.write(pan_msg.encode('utf-8'))

        tilt_msg = "SET TILT " + str(tilt_speed) + "\n"
        self.ser.write(tilt_msg.encode('utf-8'))

        msg = spin + "\n"
        self.ser.write(msg.encode('utf-8'))

        msg = safety + "\n"
        self.ser.write(msg.encode('utf-8'))

        msg = fire + "\n"
        self.ser.write(msg.encode('utf-8'))

        time.sleep(2)

    def process_image(self, msg):
        """ Process image messages from ROS and stash them in an attribute
            called cv_image for subsequent processing """
        self.cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        
        self.frame_width = self.cv_image.shape[1]
        self.frame_height = self.cv_image.shape[0]

    def find_centroids(self):
    # calculate moments of binary image
        M = cv2.moments(self.binary_image)
        # calculate x,y coordinates of center
        if M["m00"] != 0:
            cX = int(M["m10"] / M["m00"])
            cY = int(M["m01"] / M["m00"])
        else:
            cX, cY = 0, 0

        self.centroid = [cX, cY]


        # put text and highlight the center
        cv2.circle(self.cv_image, (cX, cY), 5, (0, 255, 255), -1)
        cv2.putText(self.cv_image, "centroid", (cX - 25, cY - 25),cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)
        
        


    def process_centroid(self):
        """ Input: Centroid Coordinates
            Output: Motor Commands """
            
        pan_cmd = Float64()
        tilt_cmd = Float64()
        centroid_x = self.centroid[0]
        centroid_y = self.centroid[1]

         #PID VALUES TO BE TUNED
        # PID for real life
        # self.pan_pid  = PID(Kp = 0.5, Ki = 0, Kd = 0, setpoint = (self.frame_width / 2))    # PID controller for steering
        # self.tilt_pid = PID(Kp = 0.5, Ki = 0, Kd = 0, setpoint = (self.frame_height / 2))  # PID controller for linear velocity
        
        # velocities for real life
        # pan_cmd = self.pan_pid(centroid_x)
        # tilt_cmd = self.tilt_pid(centroid_y)



        # PID simulation
        self.pan_pid  = PID(Kp = 0.5, Ki = 0, Kd = 0, setpoint = self.centroid[0])    # PID controller for steering
        self.tilt_pid = PID(Kp = 0.5, Ki = 0, Kd = 0, setpoint = self.centroid[1])  # PID controller for linear velocity

        #for simulation
        pan_cmd = self.pan_pid(self.aim[0])
        tilt_cmd = self.tilt_pid(self.aim[1])
        msg = [pan_cmd, tilt_cmd]
        # self.send_pan_and_tilt_msg(msg)

        # update aim with pan and tilt velocities
        # d = v* t
        diff = time.time() - self.start
        self.aim[0] = int(self.aim[0] + (pan_cmd * diff))
        self.aim[1] = int(self.aim[1] + (tilt_cmd * diff))
        
        
        # start timer for locking in
        if (self.aim[0] - self.aim_box_radius <= self.centroid[0] <= self.aim[0] + self.aim_box_radius) and \
            (self.aim[1] - self.aim_box_radius <= self.centroid[1] <= self.aim[1] + self.aim_box_radius):
            print("within radius")
            if self.start_timer == 0.0:
                self.start_timer = time.time()
            current = time.time()
            if 0.95 <= current - self.start_timer <= 1.05:
                self.clock += 1.0
                print("clock updated")
                if self.clock <= 3.0:
                    cv2.putText(self.cv_image, "Timer: " + str (self.clock), (self.frame_width - 50, self.frame_height + 50),cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 2)
                    print("timer up")
                else:
                    cv2.putText(self.cv_image, "FIRE" + str (self.clock), (self.frame_width / 2, self.frame_height / 2),cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 3)
                    print("Fired!")
            # timer block

        self.start = time.time()
        # put text and highlight the aim
        cv2.circle(self.cv_image, (self.aim[0], self.aim[1]), 5, (255, 0, 0), -1)
        cv2.putText(self.cv_image, "aim", (self.aim[0] - 25, self.aim[1] - 25),cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)
        # Draw a rectangle with blue line borders of thickness of 2 px
        start_point = (self.aim[0] - self.aim_box_radius, self.aim[1] - self.aim_box_radius)
        end_point = (self.aim[0] + self.aim_box_radius, self.aim[1] + self.aim_box_radius)
        cv2.rectangle(self.cv_image, start_point, end_point, (255, 0, 0), 2)

   

    
    def run_loop(self):
        # NOTE: only do cv2.imshow and cv2.waitKey in this function 
        
        # get laptop image
        ret, frame = self.cap.read()
        self.cv_image = cv2.resize(frame, None, fx=0.5, fy=0.5, interpolation=cv2.INTER_AREA)
        
        
        if not self.cv_image is None:
            self.binary_image = cv2.inRange(self.cv_image, (0,0,115), (137,61,255))
            # self.binary_image = cv2.inRange(self.cv_image, (self.blue_lower_bound,self.green_lower_bound,self.red_lower_bound), (self.blue_upper_bound,self.green_upper_bound,self.red_upper_bound))
            print(self.cv_image.shape)

            msg = self.find_centroids()
            self.process_centroid()
            # self.centroid_pub.publish(msg)
            
            # shows windows
            cv2.imshow('video_window', self.cv_image)
            cv2.imshow('binary_window', self.binary_image)
            if hasattr(self, 'image_info_window'):
                cv2.imshow('image_info', self.image_info_window)
            cv2.waitKey(5)
    
    

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