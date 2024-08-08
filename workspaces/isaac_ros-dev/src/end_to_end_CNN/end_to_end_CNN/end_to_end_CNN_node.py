import cv2
import torch 
import numpy as np  

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge

from end_to_end_CNN_node import SelfDrivingCarCNN

class end_to_end_CNN_node(Node):
    def __init__(self):
        super().__init__("End_to_End_CNN")
        self.image = None 

        # Subscribe to the front camera 
        self.image_subscriber = self.create_subscription(Image, '/video/front_camera', self.image_callback, 2)

        # Create a publisher for the steering commands
        self.nn_steering_ = self.create_publisher(Image, '/end-to-end/steering', 2)
        self.drive_commands_timer = self.create_timer(.2, self.interference_callback)

        # Init the End to End Convolutional Neural Network
        self.end_to_end_CNN = SelfDrivingCarCNN()
        self.end_to_end_CNN.load_state_dict(torch.load(""))
        self.end_to_end_CNN.eval()

        self.bridge: CvBridge = CvBridge() 

        if torch.cuda.is_available():
            self.device = torch.device('cuda:0')
        else:
            self.device = torch.device('cpu')
        print('using device:', self.device)
        self.SelfDrivingCarCNN.to(self.device)

    def image_callback(self, msg: sensor_msgs.msg.Image): 
        image = self.bridge.imgmsg_to_cv2(msg)

        resize_image = cv2.resize(image, (200, 66)) 

        # Convert the image to YUV color space
        yuv_image = cv2.cvtColor(resized_image, cv2.COLOR_BGR2YUV)
    
        # Normalize the image
        normalized_image = (yuv_image / 255.0) - 0.5
    
        # Convert to PyTorch tensor
        input_image = torch.from_numpy(normalized_image).float()
    
        # Reorder dimensions to match PyTorch's expected input format: (batch_size, channels, height, width)
        input_image = input_image.permute(2, 0, 1)  # Convert to (3, 66, 200)
    
        # Add batch dimension
        self.image = input_image.unsqueeze(0)  # Convert to (1, 3, 66, 200)
        
    
    def interference_callback(self):
        # Convert the image to YUV color space
        yuv_image = cv2.cvtColor(resized_image, cv2.COLOR_BGR2YUV)
    
        # Normalize the image
        normalized_image = (yuv_image / 255.0) - 0.5
    
        # Convert to PyTorch tensor
        input_image = torch.from_numpy(normalized_image).float()
    
        # Reorder dimensions to match PyTorch's expected input format: (batch_size, channels, height, width)
        input_image = input_image.permute(2, 0, 1)  # Convert to (3, 66, 200)
    
        # Add batch dimension
        input_image = input_image.unsqueeze(0)  # Convert to (1, 3, 66, 200)

        steering_value = None 
        with torch.no_grad(): 
            steering_value = self.end_to_end_CNN(input_image)
        

        self.nn_steering_.publish(steering_value)
        
        

def main(args=None):
    rclpy.init(args=args)
    end_to_end_CNN = end_to_end_CNN_node()
    rclpy.spin(end_to_end_CNN)
    end_to_end_CNN.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()