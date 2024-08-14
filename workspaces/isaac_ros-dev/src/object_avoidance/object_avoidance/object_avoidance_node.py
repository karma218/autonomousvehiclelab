#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

class ObjectAvoidanceNode(Node):
    def __init__(self):
        super().__init__("object_avoid")
        self.logger = self.get_logger()
        self.front_camera_subscription = self.create_subscription(Image, '/video/front_camera', self.front_camera_callback, 1)
        
    def front_camera_callback(self, msg):
        pass

def main(args=None):
    rclpy.init(args=args)
    node = ObjectAvoidanceNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__mian__":
    main()