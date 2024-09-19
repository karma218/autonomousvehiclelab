#!/usr/bin/env python3 
import rclpy 
from rclpy.node import Node  
import cv2
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
import json

PI = 3.1415
FRAME_WIDTH = 640
FRAME_HEIGHT = 480

camera_dict = dict()

#loads camera configs (json file) into camera_dict
with open('/workspaces/isaac_ros-dev/configs/camera_configs/cameras.json') as camera_json:
    camera_dict = json.load(camera_json)

#defines a class inheriting ros2s node class
class ImageStitching(Node):

    '''      
        __init__  
            Initalizes Nodes 
            @Publishers
                stitched_publisher_:  /video/stitched_frames
                left_publisher_: /video/left_camera
                right_publisher_: /video/right_camera

            @Subscribers
                front_subscription: /video/front_camera
                back_subscription: /video/back_camera
            
        __del__ 
            @releases
                left_fisheye
                right_fisheye
    '''
    def __init__(self):
        super().__init__("image_stitching")

        #creates publishers for stitched and individual camera images, 1=queue size
        self.stitched_publisher_ = self.create_publisher(Image, '/video/stitched_frames', 1)
        self.left_publisher_ = self.create_publisher(Image, '/video/left_camera', 1)
        self.right_publisher_ = self.create_publisher(Image, '/video/right_camera', 1)

        #calls callback functions when new messages are recieved
        self.front_subscription = self.create_subscription(Image, '/video/front_camera', self.front_callback, 1)
        self.back_subscription = self.create_subscription(Image, '/video/back_camera', self.back_callback, 1)

        #calls stitching_callback 20 times per second
        timer_period = 0.05
        self.timer = self.create_timer(timer_period, self.stitching_callback)

        #stores velocity and steering angle
        self.vel = 0.0
        self.steering_angle = 0.0

        #initialize video capture objects for left and right fisheye cameras
        self.left_fisheye = cv2.VideoCapture(camera_dict['/dev/left_fisheye'], cv2.CAP_V4L2) # Left
        self.right_fisheye = cv2.VideoCapture(camera_dict['/dev/right_fisheye'], cv2.CAP_V4L2) # Right

        # sets the camera width to const 640 and height to const 480
        self.left_fisheye.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
        self.left_fisheye.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)
        self.right_fisheye.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
        self.right_fisheye.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)

        # allows CvBridge to convert between ROS and OpenCV images
        self.bridge = CvBridge()
        # front/back/stitched store image data (empty initally), nums representing diff resolutions
        self.front_image = np.empty([240, 320, 3]).astype('uint8')
        self.back_image = np.empty([240, 320, 3]).astype('uint8')
        self.stitched_image = np.empty([960, 1920, 3]).astype('uint8') 

    #destructor for imagestitching
    def __del__(self):
        try:
            self.left_fisheye.release()
            self.right_fisheye.release()
        
        except Exception as e:
            print("Releasing video devices exception: ", e)

    
    def front_callback(self, msg) -> None:
        '''
            Description:
                converts front cam ros message to openCV image and resizes to 640x480
            @Params: 
                msg: contains the image message from ros to convert to openCV
            @Return: 
                None
        '''
        self.front_image = self.bridge.imgmsg_to_cv2(msg)
        self.front_image = cv2.resize(self.front_image, dsize=(640,480), interpolation=cv2.INTER_LINEAR)
        

        '''
            Description:
                converts back cam ros message to openCV image and resizes to 640x480
            @Params: 
                msg: contains the image message from ros to convert to openCV
            @Return: 
                None
        '''
    def back_callback(self, msg) -> None:
        self.back_image = self.bridge.imgmsg_to_cv2(msg)
        self.back_image = cv2.resize(self.back_image, dsize=(640,480), interpolation=cv2.INTER_LINEAR)


        '''
            Description:
                images from left, right, and back are stitched together, 
                 resized and published to /video/stitched_frames
            @Params: 
                None (besides self)
            @Return: 
                None
        '''
    def stitching_callback(self) -> None:

        try:
            stitched_frames = Image()
            left_frame = Image()
            right_frame = Image()

            _, left_image = self.left_fisheye.read()
            _, right_image = self.right_fisheye.read()
        
            #np.pad pads back image to match resolution
            back_image_padded = np.pad(self.back_image, pad_width=((0,0),(640,640),(0,0)))
            #np.concatenate combines images horizontally/vertically
            merged_frames = np.concatenate((left_image, self.front_image, right_image), axis=1)
            merged_frames = np.concatenate((merged_frames, back_image_padded), axis=0)
            #np.resize resizes frame to match resolution
            merged_frames = cv2.resize(merged_frames, dsize=(720,480), interpolation=cv2.INTER_LINEAR)
            #converts ros message to openCV image
            stitched_frames = self.bridge.cv2_to_imgmsg(merged_frames, 'bgr8')
            left_frame = self.bridge.cv2_to_imgmsg(left_image, 'bgr8')
            right_frame = self.bridge.cv2_to_imgmsg(right_image, 'bgr8')

            #publishes each frame
            self.stitched_publisher_.publish(stitched_frames)
            self.left_publisher_.publish(left_frame)
            self.right_publisher_.publish(right_frame)

        except Exception as e:
            print(e)

def main(args=None):
    # starts ros2 communication, it is mandatory 
    rclpy.init(args=args)
    image_stitching_node = ImageStitching()

    # pauses program, keeps node running, so no shutdown
    rclpy.spin(image_stitching_node)
    rclpy.shutdown()

#calls main to start node
if __name__=="__main__":
    main()