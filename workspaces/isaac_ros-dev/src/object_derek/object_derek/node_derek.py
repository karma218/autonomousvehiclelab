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
    # def __init__(self):
    #     super().__init__("object_avoid")
    #     self.logger = self.get_logger()
    #     self.bridge = CvBridge()
    #     self.speed = 0.25  # Speed in feet per second

    #     self.front_publisher_ = self.create_publisher(geometry_msgs.msg.Twist, '/object_detection/cmd_vel', 1)
    #     self.video_publisher = self.create_publisher(Image, '/video/object_detection', 1)

    #     self.front_camera_subscription = self.create_subscription(Image, '/video/front_camera', self.front_camera_callback, 1)
    #     self.is_performing_turn = False
        
    # def front_camera_callback(self, msg):
    #     speed = 0.25
    #     angle = 0.0
    #     self.get_logger().info('front_camera_callback')
    #     try:
    #         image_array = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
    #         image, result = detect_object(image_array)

    #         video_frame = self.bridge.cv2_to_imgmsg(image, 'bgr8')
    #         self.video_publisher.publish(video_frame)

    #         if result == 'left':
    #             angle = -0.8
    #         elif result == 'right':
    #             angle = 0.8
    #         else:
    #             angle = 0.0

    #         twist = geometry_msgs.msg.Twist()
    #         twist.linear.x = speed
    #         twist.linear.y = 0.0
    #         twist.linear.z = 0.0
    #         twist.angular.x = 0.0
    #         twist.angular.y = 0.0
    #         twist.angular.z = angle

    #         self.front_publisher_.publish(twist)
    #         self.video_publisher.publish(video_frame)


    #     except Exception as e:
    #         self.get_logger().error(f"Error in front_camera_callback: {e}")

            
# ------------------------------------------Object Derek from Derek ---------------------------------------------------------------------
    #         if result == 'STOP':
    #             self.avoding_object()
    #             # self.stop_robot()
    #             # self.video_publisher.publish(video_frame)

    #         # # elif result == 'TURN':
    #         #     turn_angles = [451, 512, 572, 512, 572, 512, 451, 512] 
    #         #     # 512 -> 451
    #         #     turn_angles = [[0, 30], [30, 0]]
    #         #     for start, end in turn_angles:
    #         #         if start > end:
    #         #             direction = -1
    #         #         else:
    #         #             direction = 1
    #         #         for angle in range(start, end, direction):
    #         #             a = angle / 100
    #         #             self.get_logger().info(f'Angle: {a}')
    #         #             self.perform_turn(a)
    #         #         # self.travel_distance(1)  # Travel 5 feet
                    
                    
    #         else:
    #             self.move_forward()
    #             # self.video_publisher.publish(video_frame)

    #     except Exception as e:
    #         self.get_logger().error(f"Error in front_camera_callback: {e}")

    # def avoding_object(self):
    #     self.is_performing_turn = True
    #     turn_angles = [[0.5,  15], [0.7,  15], [1.0,  150],  [0.7,  15], [0.5,  15]]
    #     # time_start = time.time()
    #     # time_curr = time.time()
    
    #     for angle, number in turn_angles:
    #         # if start > end:
    #         #     direction = -1
    #         # else:
    #         #     direction = 1
    #         for _ in range(number):
    #             # a = angle / 100
    #             self.get_logger().info(f'Angle: {angle}')
    #             self.perform_turn(angle)
    #             time.sleep(0.1) # would be too slow, keep small 0.3
    #     self.is_performing_turn = False


    # def stop_robot(self):
    #     twist = geometry_msgs.msg.Twist()
    #     twist.linear.x = 0.0
    #     twist.angular.z = 0.0
    #     self.front_publisher_.publish(twist)

    # def move_forward(self):
    #     twist = geometry_msgs.msg.Twist()
    #     twist.linear.x = self.speed
    #     twist.angular.z = 0.0
    #     self.front_publisher_.publish(twist)

    # def perform_turn(self, angle):
    #     twist = geometry_msgs.msg.Twist()
    #     twist.linear.x = self.speed
    #     twist.angular.z = angle / 1.0
    #     self.front_publisher_.publish(twist)

    # def travel_distance(self, distance):
    #     # Calculate the duration to travel the given distance
    #     duration = distance / self.speed  # Duration in seconds
    #     self.get_logger().info(f"Travelling for {duration} seconds to cover {distance} feet.")
    #     start_time = time.time()
        
    #     while time.time() - start_time < duration:
    #         # Keep moving forward for the duration
    #         self.move_forward()
    #         time.sleep(0.1)  # Small sleep to avoid busy-waiting

    #     self.stop_robot()  # Stop the robot after travelling

# ------------------------- End Object Derek from Derek ---------------------------

    def __init__(self):
        super().__init__("object_avoid")
        self.logger = self.get_logger()
        self.bridge = CvBridge()
        self.speed = 0.25  # Speed in feet per second

        self.front_publisher_ = self.create_publisher(geometry_msgs.msg.Twist, '/twist_mux/cmd_vel', 1)
        self.video_publisher = self.create_publisher(Image, '/video/object_detection', 1)

        self.front_camera_subscription = self.create_subscription(Image, '/video/front_camera', self.front_camera_callback, 1)
        
    def front_camera_callback(self, msg):
        self.get_logger().info('front_camera_callback')
        try:
            image_array = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            image, result = detect_object(image_array)

            video_frame = self.bridge.cv2_to_imgmsg(image, 'bgr8')

            self.get_logger().info(f"{result}")
            if result == 'STOP':
                self.stop_robot()
                # self.video_publisher.publish(video_frame)

            # elif result == 'TURN':
                turn_angles = [-0.8, 0.0, 0.8, 0.0, 0.8, 0.0, -0.8, 0.0] 
                # turn_angles = [451.0, 512.0, 572.0, 512.0, 572.0, 512.0, 451.0, 512.0] 
                for this_turn in turn_angles:
                    self.get_logger().info('performing turn')
                    self.perform_turn(this_turn, 5)  # Turn to angle and move for 5 seconds
                    # self.video_publisher.publish(video_frame)
                    
            else:
                self.move_forward()
            
            self.video_publisher.publish(video_frame)

        except Exception as e:
            self.get_logger().error(f"Error in front_camera_callback: {e}")

    def stop_robot(self):
        twist = geometry_msgs.msg.Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.get_logger().info('Stopping....')
        self.front_publisher_.publish(twist)

    def move_forward(self):
        twist = geometry_msgs.msg.Twist()
        twist.linear.x = self.speed
        twist.angular.z = 0.0
        self.get_logger().info('Forward....')
        self.front_publisher_.publish(twist)

    def perform_turn(self, angle, duration):
        twist = geometry_msgs.msg.Twist()
        twist.linear.x = self.speed
        twist.angular.z = angle
        self.get_logger().info(f'Performing turn....{angle}')
        # Move for the specified duration
        self.get_logger().info(f"Turning and moving for {duration} seconds.")
        self.front_publisher_.publish(twist)
        start_time = time.time()
        
        while time.time() - start_time < duration:
            # Keep turning for the duration
            self.front_publisher_.publish(twist)
            time.sleep(0.1)  # Small sleep to avoid busy-waiting
        self.stop_robot()  # Stop the robot after the turn and move duration

def main(args=None):
    rclpy.init(args=args)
    node = ObjectAvoidanceNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
