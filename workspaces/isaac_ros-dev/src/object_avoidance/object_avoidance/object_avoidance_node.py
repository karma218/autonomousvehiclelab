#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from object_avoidance.object_avoidance import detect_object
from cv_bridge import CvBridge
import geometry_msgs.msg

class ObjectAvoidanceNode(Node):
    def __init__(self):
        super().__init__("object_avoid")
        self.logger = self.get_logger()

        self.front_publisher_ = self.create_publisher(geometry_msgs.msg.Twist, '/object_detection/cmd_vel', 1)

        self.front_camera_subscription = self.create_subscription(Image, '/video/front_camera', self.front_camera_callback, 1)
        self.bridge = CvBridge()
        
    def front_camera_callback(self, msg):
        speed = 0.25
        print('front_camera_callback')
        image_array = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        if detect_object(image_array) == 'STOP':
            speed = 0.0
        twist = geometry_msgs.msg.Twist()
        twist.linear.x = speed
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = 0.0
        # print(f'key pressed: {key},', vels(speed, turn))
        self.front_publisher_ .publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = ObjectAvoidanceNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__mian__":
    main()