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
        super().__init__("data_logger_py")
        self.file_count = 0 
        self.image_count = 0
        if not os.path.exists("logging/logging_data"): 
                return
        
        if not os.path.exists("logging/image_data"): 
                return 
                
        for file in os.listdir("logging/logging_data"): 
                file_txt = os.path.join("logging/logging_data/", file)
                if os.path.exists(file_txt) and not file.startswith("."): 
                        self.file_count += 1
        
        for file in os.listdir("logging/image_data"): 
                file_txt = os.path.join("logging/image_data/", file)
                if os.path.exists(file_txt) and not file.startswith("."): 
                        self.image_count += 1 
        self.image_count /= 3

        self.new_file_txt = open("logging/logging_data/log_file_" + str(self.file_count) + ".txt", "w+") 

        self.steering_subscription_ = self.create_subscription(Twist,'/twist_mux/cmd_vel', self.steering_callback, 1)
        self.front_camera_subscription_ = self.create_subscription(Image, '/video/front_camera', self.front_camera_callback, 1)
        self.left_camera_subscription_ = self.create_subscription(Image, '/video/left_camera', self.left_camera_callback, 1)
        self.right_camera_subscription_ = self.create_subscription(Image, '/video/right_camera', self.right_camera_callback, 1)

        self.front_frame_ = None
        self.left_frame_ = None
        self.right_frame_ = None
        self.steering_value_ = None

        self.bridge = CvBridge()
        self.get_logger().info("Start up")


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
        
        steering_angle = (msg.angular.z)
        steering_angle_int = int(steering_angle*61.0+512.0)
       
        left_frame_cv2 = self.bridge.imgmsg_to_cv2(self.left_frame_, 'bgr8') 
        right_frame_cv2 = self.bridge.imgmsg_to_cv2(self.right_frame_, 'bgr8') 
        front_frame_cv2 = self.bridge.imgmsg_to_cv2(self.front_frame_, 'bgr8') 

        if left_frame_cv2 is None or right_frame_cv2 is None or front_frame_cv2 is None: 
                return
        
        self.image_count += 1

        timestamp = time.time()

        cv2.imwrite("logging/image_data/left_frame_" + str(self.image_count) + ".jpg", left_frame_cv2) 
        cv2.imwrite("logging/image_data/right_frame_" + str(self.image_count) + ".jpg", right_frame_cv2) 
        cv2.imwrite("logging/image_data/front_frame_" + str(self.image_count) + ".jpg",  front_frame_cv2) 


        self.new_file_txt.write(str(timestamp) + " left_frame_" + str(self.image_count) + " right_frame_" + str(self.image_count) 
                + " front_frame_" + str(self.image_count) + " " + str(steering_angle_int) + "\n")

        self.get_logger().info("Image added")




         
def main(args=None):
    rclpy.init(args=args)    # strts ros2 communication, it is mandatory 
    data_logger_py_node = data_logger_py()
    rclpy.spin(data_logger_py_node)        # pauses program, keeps node running, so no shutdown
    rclpy.shutdown()

if __name__=="__main__":
    main()