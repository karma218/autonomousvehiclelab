import cv2
import torch 
import numpy as np  

import rclpy
import torchvision.transforms as transforms
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge

from end_to_end_CNN import end_to_end_CNN_model

transform = transforms.Compose([
    transforms.ToPILImage(), 
    transforms.Resize((66, 200)),
    transforms.ToTensor(), 
    transforms.Normalize(mean=[0.5, 0.5, 0.5], std=[0.5, 0.5, 0.5])
])

class end_to_end_CNN_node(Node):
    def __init__(self):
        super().__init__("End_to_End_CNN")
        # Create Object for Twist Commands 
        self.drive_twist_: Twist = Twist() 
        self.bridge_: CvBridge = CvBridge() 

        self.front_image_ = None
        self.device_ = None

        # Subscribe to the front camera 
        self.front_image_subscriber_ = self.create_subscription(Image, '/video/front_camera', self.front_image_callback, 2)

        # Create a publisher for the steering commands
        self.nn_steering_ = self.create_publisher(Twist, '/twist_mux/cmd_vel', 2)
        self.drive_commands_timer_ = self.create_timer(.2, self.model_callback)


        # Init the End to End Convolutional Neural Network
        self.end_to_end_CNN_: end_to_end_CNN_model.SelfDrivingCarCNN = end_to_end_CNN_model.SelfDrivingCarCNN()
        self.end_to_end_CNN_.load_state_dict(torch.load('/avlcode/workspaces/isaac_ros-dev/src/end_to_end_CNN/models/checkpoints/model.pth'))
        self.end_to_end_CNN_.eval()


        if torch.cuda.is_available():
            self.device_ = torch.device('cuda:0')
        else:
            self.device_ = torch.device('cpu')

        print('using device:', self.device_)
        self.end_to_end_CNN_.to(self.device_)

    def front_image_callback(self, msg: sensor_msgs.msg.Image) -> None: 
        img = self.bridge.imgmsg_to_cv2(msg)
        self.front_image_ = img

    def model_callback(self) -> None:

        if self.front_image_ is None: 
            self.get_logger().error(f'Error when attempting to get Front Camera')
            return

        front_frame = self.front_image_ 

        steering_value = None 
        with torch.no_grad(): 
            front_frame = transform(front_frame)
            front_frame = torch.unsqueeze(front_frame, 1)
            front_frame = front_frame.permute(1, 0, 2, 3)

            steering_value = self.end_to_end_CNN_(front_frame).squeeze() 
        
        self.get_logger().info("Steering value: " + str(steering_value.cpu().numpy().item()))

        self.drive_twist_.angular.z = steering_value.cpu().numpy().item()
        self.drive_twist_.linear.x = .25

        self.nn_steering_.publish(drive_twist)
        
        

def main(args = None):
    rclpy.init(args=args)

    end_to_end_CNN = end_to_end_CNN_node()
    rclpy.spin(end_to_end_CNN)
    end_to_end_CNN.destroy_node()

    rclpy.shutdown()

if __name__ == '__main__':
    main()