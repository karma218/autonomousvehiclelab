#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from object_avoidance.object_avoidance import detect_object
from cv_bridge import CvBridge
import geometry_msgs.msg
import cv2

class ObjectAvoidanceNode(Node):
    def __init__(self):
        super().__init__("object_avoid")
        self.logger = self.get_logger()
        self.bridge = CvBridge()

        self.front_publisher_ = self.create_publisher(geometry_msgs.msg.Twist, '/object_detection/cmd_vel', 1)
        self.video_publisher = self.create_publisher(Image, '/video/object_detection', 1)

        self.front_camera_subscription = self.create_subscription(Image, '/video/front_camera', self.front_camera_callback, 1)
        
    def front_camera_callback(self, msg):
        speed = 0.25
        self.get_logger().info('front_camera_callback')
        try:
            image_array = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            image, result = detect_object(image_array)

            video_frame = self.bridge.cv2_to_imgmsg(image, 'bgr8')
            # Handle OpenCV GUI functions outside of ROS callback to avoid potential issues
            # if cv2.getWindowProperty('Processed Image', cv2.WND_PROP_VISIBLE) < 1:
            #     cv2.imshow('Processed Image', image)
            #     cv2.waitKey(1)  # Process GUI events

            if result == 'STOP':
                speed = 0.0

            twist = geometry_msgs.msg.Twist()
            twist.linear.x = speed
            twist.linear.y = 0.0
            twist.linear.z = 0.0
            twist.angular.x = 0.0
            twist.angular.y = 0.0
            twist.angular.z = 0.0
            
            self.front_publisher_.publish(twist)
            self.video_publisher.publish(video_frame)
            
        
        except Exception as e:
            self.get_logger().error(f"Error in front_camera_callback: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = ObjectAvoidanceNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__mian__":
    main()