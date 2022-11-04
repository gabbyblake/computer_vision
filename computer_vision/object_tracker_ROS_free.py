from threading import Thread
import time
import serial
import numpy as np
from copy import deepcopy
import cv2
import os
from simple_pid import PID


class ObjectTracker():

    def __init__(self):
        """ Initialize the object tracker """

        # OpenCV stuff
        self.cv_image = None                        # the latest image from the camera
        self.cap = cv2.VideoCapture(4)
        if not self.cap.isOpened():
            raise IOError("Cannot open webcam")

        # facial recognition
        cascPath = os.path.dirname(cv2.__file__)+"/data/haarcascade_frontalface_default.xml"
        self.faceCascade = cv2.CascadeClassifier(cascPath)

        # serial comm. with the Sentry driver Pico board
        self.ser = serial.Serial(
            port='/dev/ttyACM1', # Change this according to connection methods, e.g. /dev/ttyUSB0
            baudrate = 115200,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            bytesize=serial.EIGHTBITS,
            timeout=1
        )

        # initial guess of frame dimensions
        self.frame_width  = self.cap.get(3)  # float `width`
        self.frame_height = self.cap.get(4)  # float `height`
        print(f"frame width: {self.frame_width} frame height: {self.frame_height}")
        # self.frame_width = 320 # px
        # self.frame_height = 240 # px
        
        # Target tracking params
        self.aim_box_radius = 20
        self.lock_timer = 0.0

        self.centroid = [int(self.frame_width//2), int(self.frame_height//2)]
        self.aim = self.centroid

       

        # PIDs for simulator
        self.pan_pid  = PID(Kp = -0.001, Ki = 0, Kd = 0.0001, setpoint = (self.frame_width / 2),  output_limits=[-1.0, 1.0])    # PID controller for steering
        self.tilt_pid = PID(Kp = 0.001, Ki = 0, Kd = 0.0001, setpoint = (self.frame_height / 2), output_limits=[-1.0, 1.0])  # PID controller for linear velocity
        

        # Countdown to fire timer
        self.start = time.time()
        self.timer_start = -1.0
        self.clock = 0.0
        self.fired = False

        loop_thread = Thread(target=self.loop_wrapper)
        loop_thread.start()

    def send_ser_line(self, line):
        """
        Send a line to the pico via serial connection

        Args:
            line (string): 1-line message to send, no return character
        """
        self.ser.write(bytes(line + "\n", "utf_8"))      # write a string

    def send_motor_cmd(self, motor, speed):
        cmd = f"SET {motor} {speed}"
        self.send_ser_line(cmd)
        # DEBUG
        print(speed)

    def send_spin_cmd(self, val):
        msg = "SPIN " + ("UP" if val else "DOWN")
        self.ser.write(msg.encode('utf-8'))

    def send_safety_cmd(self, val):
        msg = "SAFETY " + ("ON" if val else "OFF")
        self.send_ser_line(msg)

    def send_fire_cmd(self):
        msg = "FIRE"
        # DEBUG:
        # self.send_ser_line(msg)
        self.fired = True
        print("sent message to fire")

    def find_faces(self):
        # Capture frame-by-frame
        gray = cv2.cvtColor(self.cv_image, cv2.COLOR_BGR2GRAY)

        faces = self.faceCascade.detectMultiScale(
            gray,
            scaleFactor=1.1,
            minNeighbors=5,
            minSize=(30, 30),
            flags=cv2.CASCADE_SCALE_IMAGE
        )
        self.centroid = [int(self.frame_width/2), int(self.frame_height/2)]
        # Draw a rectangle around the faces
        for (x, y, w, h) in faces:
            if w*h > 10000:
                cv2.rectangle(self.cv_image, (x, y), (x+w, y+h), (0, 125, 0), 2)
                self.centroid = [int(x+(w/2)), int(y+(h/2))]
            else:
                self.centroid = [int(self.frame_width/2), int(self.frame_height/2)]
        clear = np.any(faces)

        # Draw rect around first detected face
        if clear:
            (x, y, w, h) = faces[0]
            if w*h > 10000:
                cv2.rectangle(self.cv_image, (x, y), (x+w, y+h), (0, 255, 0), 2)
                self.centroid = [int(x+(w/2)), int(y+(h/2))]
            else:
                self.centroid = [int(self.frame_width/2), int(self.frame_height/2)]

        # Print out centroid
        [cX, cY] = self.centroid
        if cX > 25:
            print([cX, cY])
            # put text and highlight the center
            cv2.circle(self.cv_image, (cX, cY), 5, (0, 255, 255), -1)
            cv2.putText(self.cv_image, "centroid", (cX - 25, cY - 25),cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)

    def find_shapes(self):
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

         #PID VALUES TO BE TUNED
        # PID for real life
        # self.pan_pid  = PID(Kp = 0.8, Ki = 0, Kd = -0.25, setpoint = (self.frame_width / 2))    # PID controller for steering
        # self.tilt_pid = PID(Kp = 0.8, Ki = 0, Kd = -0.25, setpoint = (self.frame_height / 2))  # PID controller for linear velocity
        
        # velocities for real life
        pan_speed = self.pan_pid(self.centroid[0])
        tilt_speed = self.tilt_pid(self.centroid[1])

        # PID simulation
        # self.pan_pid  = PID(Kp = 0.8, Ki = 0, Kd = -0.25, setpoint = self.centroid[0])    # PID controller for steering
        # self.tilt_pid = PID(Kp = 0.8, Ki = 0, Kd = -0.25, setpoint = self.centroid[1])  # PID controller for linear velocity

        # #for simulation
        # pan_speed = self.pan_pid(self.aim[0])
        # tilt_speed = self.tilt_pid(self.aim[1])

        # send pan and tilt cmds
        msg = [pan_speed, tilt_speed]
        self.send_motor_cmd("PAN", pan_speed)
        self.send_motor_cmd("TILT", tilt_speed)

        # # (SIMULATOR) update aim with pan and tilt velocities
        # # d = v* t
        # diff = time.time() - self.start
        # self.aim[0] = int(self.aim[0] + (pan_speed * diff))
        # self.aim[1] = int(self.aim[1] + (tilt_speed * diff))
        
        
        # start timer for locking in
        if (self.aim[0] - self.aim_box_radius <= self.centroid[0] <= self.aim[0] + self.aim_box_radius) and \
            (self.aim[1] - self.aim_box_radius <= self.centroid[1] <= self.aim[1] + self.aim_box_radius):
            # print("within radius")
            if self.timer_start == -1.0:
                self.timer_start = time.time()
            current = time.time()
            timer_diff = current - self.timer_start + 1.0
            fire_threshold = 2.0
            if timer_diff <= fire_threshold + 1.0:
                # print(timer_diff)
                text = "Timer: " + str(int(timer_diff))
                cv2.putText(self.cv_image, text, (int(self.frame_width - 75), int(25)),cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 2)
                # print("timer up")
            else:
                if not self.fired:
                    cv2.putText(self.cv_image, "FIRE!", (90, int(self.frame_height / 2)),cv2.FONT_HERSHEY_SIMPLEX, 2.0, (0, 0, 255), 3)
                    self.send_fire_cmd()
                    print("Fired!")
        else:
            # print("outside_radius")
            self.timer_start = time.time()
            self.fired = False
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
        _, frame = self.cap.read()
        self.cv_image = frame
        #self.cv_image = cv2.resize(frame, None, fx=0.5, fy=0.5, interpolation=cv2.INTER_AREA)
        
        
        if not self.cv_image is None:
            self.binary_image = cv2.inRange(self.cv_image, (0,0,115), (137,61,255))
            # self.binary_image = cv2.inRange(self.cv_image, (self.blue_lower_bound,self.green_lower_bound,self.red_lower_bound), (self.blue_upper_bound,self.green_upper_bound,self.red_upper_bound))
            # print(self.cv_image.shape)

            # self.find_shapes()
            self.find_faces()
            self.process_centroid()

            
            # shows windows
            cv2.imshow('video_window', self.cv_image)
            # cv2.imshow('binary_window', self.binary_image)
            # if hasattr(self, 'image_info_window'):
                # cv2.imshow('image_info', self.image_info_window)
            cv2.waitKey(5)
    
    

    def loop_wrapper(self):
        """ This function takes care of calling the run_loop function repeatedly.
            We are using a separate thread to run the loop_wrapper to work around
            issues with single threaded executors in ROS2 """
        cv2.namedWindow("video_window")
        # cv2.namedWindow("binary_window")
        # cv2.namedWindow("image_info")
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

    


def main(args=None):
    tracker = ObjectTracker()


if __name__ == '__main__':
    main()