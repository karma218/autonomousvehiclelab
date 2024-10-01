#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from object_avoidance.object_avoidance import detect_object
from cv_bridge import CvBridge
import geometry_msgs.msg
import time
import cv2

class ObjectAvoidanceNode(Node):
    def __init__(self):
        super().__init__("object_avoid")
        
        self.is_turning = False
        self.turning_path = []
        self.turning_path_B = []
        self.logger = self.get_logger()
        self.bridge = CvBridge()

        self.front_publisher_ = self.create_publisher(geometry_msgs.msg.Twist, '/object_detection/cmd_vel', 10)
        self.video_publisher = self.create_publisher(Image, '/video/object_detection', 10)

        self.front_camera_subscription = self.create_subscription(Image, '/video/front_camera', self.front_camera_callback, 10)

    def front_camera_callback(self, msg):
        speed = 0.3
        angle = 0.0
        # self.get_logger().info('front_camera_callback')
        try:
            image_array = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            image, direction, person = detect_object(image_array)   
            video_frame = self.bridge.cv2_to_imgmsg(image, 'bgr8')
            self.video_publisher.publish(video_frame)
         
            # if statement that checks if person is false, self.is_turning is false, and the length of self.turning_path and self.turning_path_B is = 0
            if direction == 'straight' and self.is_turning == False and len(self.turning_path) == 0 and len(self.turning_path_B) == 0:
                angle = 0.0
                self.get_logger().info(f"STAG 0: GO STRAIGHT")  
               
            elif direction == 'left':
                self.is_turning = True
                angle = 0.7
                self.get_logger().info(f"STAG 1: LEFT | CNT: {len(self.turning_path)}") 
               
            elif direction == 'right':
                self.is_turning = True
                angle = -0.7
                self.get_logger().info(f"STAG 1: RIGHT | CNT: {len(self.turning_path)}") 
            
            # elif statement that checks if direction is straight, self.is_turning is true, and the length of self.turning_path and self.turning_path_B is > 0
            elif direction == 'straight' and self.is_turning == True and len(self.turning_path_B) > 0: 
                self.get_logger().info(f"STAG 2: REVERSE | CNT: {len(self.turning_path)}") 
                if len(self.turning_path) > 0:
                    self.reverse_angle(-1)
                    return
                elif len(self.turning_path) == 0:
                    self.get_logger().info(f"STAG 2: REVERSE END") 
                    angle = 0.0
                    self.is_turning = False

            elif direction == 'straight' and self.is_turning == False and person == True:
                self.get_logger().info(f"STAG 3: GO STRAIGHT") 
                angle = 0.0

            elif direction == 'straight' and self.is_turning == False and person == False and len(self.turning_path) == 0 and len(self.turning_path_B) > 0:
                # self.get_logger().info(f"STAG 3: COPY TURNING STACK 2 TIMES") 
                for twist in self.turning_path_B:
                    twist = self.copy_twist(twist)
                    twist.angular.z = -twist.angular.z
                    self.turning_path.append(twist)
                
                for twist in self.turning_path_B:
                    twist = self.copy_twist(twist)
                    self.turning_path.append(twist)
                self.get_logger().info(f"STAG 3: COPY TURNING STACK 2 TIMES | CNT: {len(self.turning_path)}") 
                self.turning_path_B = []
                angle = 0.0 
            
            elif direction == 'straight' and self.is_turning == False and person == False and len(self.turning_path) > 0 and len(self.turning_path_B) == 0:
                self.get_logger().info(f"STAG 4: REVERSE 2 TIMES | CNT: {len(self.turning_path)}") 
                self.reverse_angle(1)
                return    
             
            # self.get_logger().info(f"direction: {direction}")   
            twist = self.create_twist(speed, angle)
           
            self.front_publisher_.publish(twist)
            self.get_logger().info(f'FCP Twist: {twist}')
           
            if self.is_turning and direction in ['left', 'right']:
                self.turning_path.append(twist)
                self.turning_path_B.append(twist)
           
    
        except Exception as e:
            self.get_logger().error(f"Error in front_camera_callback: {e}")
           
    def reverse_angle(self,a):
        twist = self.turning_path.pop()
        twist.angular.z = a*twist.angular.z
        # self.get_logger().info(f"reverse: {twist}")
        self.front_publisher_.publish(twist)
        self.get_logger().info(f'FCP Twist: {twist}')
        return twist
    
    def copy_twist(self, twist):
        return self.create_twist(twist.linear.x, twist.angular.z)

    def create_twist(self, speed, angle):
        twist = geometry_msgs.msg.Twist()
        twist.linear.x = speed
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = angle
        return twist

        
    # def reverse_angle2(self,a):
    #     twist = self.turning_path_B.pop()
    #     twist.angular.z = a*twist.angular.z
    #     self.get_logger().info(f"reverse: {twist}")
    #     self.front_publisher_.publish(twist) 
    #     return twist 
        
    # def front_camera_callback(self, msg):
    #     speed = 0.2
    #     self.get_logger().info('front_camera_callback')
    #     try:
    #         image_array = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
    #         image, result = detect_object(image_array)

    #         video_frame = self.bridge.cv2_to_imgmsg(image, 'bgr8')
    #         self.video_publisher.publish(video_frame)


                
    #         if result == 'left':
    #             self.is_turning = True
    #             angle = 0.7
                
    #         elif result == 'right':
    #             self.is_turning = True
    #             angle = -0.7 
            
    #         else:
    #             self.is_turning = False
    #             angle = 0.0 
                
    #         if self.is_turning == False and len(self.turning_path) > 0:
    #             self.reverse_angle()
    #             return
            
    #         self.get_logger().info(f"direction: {result}")

    #         twist = geometry_msgs.msg.Twist()
    #         twist.linear.x = speed
    #         twist.linear.y = 0.0
    #         twist.linear.z = 0.0
    #         twist.angular.x = 0.0
    #         twist.angular.y = 0.0
    #         twist.angular.z = angle
            
    #         self.front_publisher_.publish(twist)
            
    #         if self.is_turning:
    #             self.turning_path.append(twist)
            
        
    #     except Exception as e:
    #         self.get_logger().error(f"Error in front_camera_callback: {e}")
            
    # def reverse_angle(self):
    #     twist = self.turning_path.pop()
    #     twist.angular.z = -twist.angular.z
    #     self.get_logger().info(f"reverse: {twist}")
    #     self.front_publisher_.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = ObjectAvoidanceNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__mian__":
    main()