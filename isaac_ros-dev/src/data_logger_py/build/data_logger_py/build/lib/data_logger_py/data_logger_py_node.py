import rclpy
from rclpy.node import Node

#from custom_message.msg import Wheels
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import serial
import time
import sys
import math
import os


class data_logger_py(Node): 
    def __init__(self): 
        self.file_count = 0 
        self.image_count = 0
        
        if not os.path.exists("/logging/logging_data"): 
                return
        
        if not os.path.exists("/logging/image_data"): 
                return 
                
        for file in os.listdir("/logging/logging_data"): 
                file_txt = os.path.join("/logging/logging_data/logging_data", file)
                if os.path.exists(file_txt) and not file.startswith("."): 
                        file_count += 1

        self.new_file_txt = open("log_file_" + str(file_count) + ".txt") 

        self.steering_subscription_ = self.create_subscription(Twist,'/dev/video_camera', self.steering_callback, 1)
        self.front_camera_subscription_ = self.create_subscription(Image, '/dev/front_camera', self.front_camera_callback, 1)
        self.left_camera_subscription_ = self.create_subscription(Image, '/dev/left_camera', self.left_camera_callback, 1)
        self.right_camera_subscription_ = self.create_subscription(Image, '/dev/right_camera', self.right_camera_callback, 1)

        self.front_frame_ = None
        self.left_frame_ = None
        self.right_frame_ = None
        self.steering_value_ = None

    def front_camera_callback(self, msg): 
        self.front_frame_ = msg
    
    def right_camera_callback(self, msg):
        self.right_frame_ = msg
    
    def left_camera_callback(self, msg): 
        self.left_frame_ = msg
    
    def steering_callback(self, msg):
        if self.front_frame_ is None: 
                return 
        
        if self.left_frame_ is None:
                return 

        if self.right_frame_ is None:
                return 

        if self.steering_value_ is None: 
                return 

        image_count += 1
       
        left_frame_cv2 = self.bridge.imgmsg_to_cv2(self.left_frame_, 'bgr8') 
        right_frame_cv2 = self.bridge.imgmsg_to_cv2(self.right_frame_, 'bgr8') 
        front_frame_cv2 = self.bridge.imgmsg_to_cv2(self.front_frame_, 'bgr8') 

        if left_frame_cv2.empty() or right_frame_cv2.empty() or front_frame_cv2.empty(): 
                return

        timestamp = time.time()

        cv2.imwrite("left_frame_" + str(image_count), left_frame_cv2) 
        cv2.imwrite("right_frame_" + str(image_count), right_frame_cv2) 
        cv2.imwrite("front_frame_" + str(image_count), front_frame_cv2) 


        self.new_file_txt.write(str(timestamp) + " left_frame_" + str(image_count) + " right_frame_" + str(image_count) 
                + " front_frame_" + str(image_count) + " " + str(steering_value))



         
