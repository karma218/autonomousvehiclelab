import rclpy
from rclpy.node import Node

#from custom_message.msg import Wheels
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
import serial
import time
import sys
import math


class data_logger_py(Node): 
    def __init__(self): 
        self.file = open("logging/logging_data/")

        self.steering_subscription = self.create_subscription(Twist,'/dev/video_camera', self.steering_callback, 1)
        self.front_camera_subscription = self.create_subscription(Image, '/dev/front_camera', self.front_camera_callback, 1)
        self.left_camera_subscription = self.create_subscription(Image, '/dev/left_camera', self.left_camera_callback, 1)
        self.right_camera_sbscription = self.create_subscription(Image, '/dev/right_camera', self.right_camera_callback, 1)

        self.front_frame_ = None
        self.left_frame_ = None
        self.right_frame_ = None

    def front_camera_callback(self, msg): 
        self.front_frame_ = msg
    
    def right_camera_callback(self, msg):
        self.right_frame_ = msg
    
    def left_camera_callback(self, msg): 
        self.left_frame_ = msg
    
    def steering_callback(self, msg):
        if 