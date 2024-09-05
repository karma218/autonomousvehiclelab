#include <memory> 
#include <string> 
#include <functional> 
#include <chrono> 

#include <sensor_msgs/image_encodings.hpp>
#include <image_transport/image_transport.h> 
#include <opencv2/imgproc/imgproc.hpp> 
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/videoio.hpp> 
#include <cv_bridge/cv_bridge.h>

#include "rclcpp/rclcpp.hpp" 
#include "std_msgs/msg/string.hpp"

#define FRAME_WIDTH 640
#define FRAME_HEIGHT 480

class CameraPublisher : public rclcpp::Node {
    public: 
        CameraPublisher() : Node("camera_publisher") {
            m_front_cam_.open(FRONT_CAM_DEV_ID, cv::CAP_V4L2);
			m_front_cam_.set(cv::CAP_PROP_FRAME_WIDTH, FRAME_WIDTH); 
			m_front_cam_.set(cv::CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT);
            if (!m_front_camd.is_open()){
                RCLCPP_ERROR(this->get_logger(), "Unable to open front camera");
                return; 
            } 

            m_right_cam_.open(RIGHT_CAM_DEV_ID, cv::CAP_V4L2);
			m_right_cam_.set(cv::CAP_PROP_FRAME_WIDTH, FRAME_WIDTH); 
			m_right_cam_.set(cv::CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT);
            if (!m_right_cam.is_open()){
                RCLCPP_ERROR(this->get_logger(), "Unable to open right camera");
                return; 
            }

            m_left_cam_.open(RIGHT_CAM_DEV_ID, cv::CAP_V4L2);
			m_left_cam_.set(cv::CAP_PROP_FRAME_WIDTH, FRAME_WIDTH); 
			m_left_cam_.set(cv::CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT);
            if (!m_left_cam.is_open()){
                RCLCPP_ERROR(this->get_logger(), "Unable to open left camera");
                return; 
            }

            m_front_camera_publisher_ = this->create_publisher<sensor_msgs::msg::Image>("/video/front_camera", 1); 
            m_right_camera_publisher_ = this->create_publisher<sensor_msgs::msg::Image>("/video/right_camera", 1); 
            m_left_camera_publisher_ = this->create_publisher<sensor_msgs::msg::Image>("/video/left_camera", 1); 

            m_front_cam_timer_ = this->create_wall_timer(500ms, 
                std::bind(&MinimalPublisher::front_camera_callback, this)); 
            m_right_cam_timer_ = this->create_wall_timer(500ms,
                std::bind(&CameraPublisher::right_camera_callback, this));
            m_left_cam_timer_ = this->create_wall_timer(500ms,
                std::bind(&CameraPublisher::left_camera_callback, this));
        } 

    private: 
        const int FRONT_CAM_DEV_ID = 0; 
        const int LEFT_CAM_DEV_ID = 0; 
        const int RIGHT_CAM_DEV_ID = 0;

        rclcpp::Publisher<sensor_msgs::msg::Image> m_front_camera_publisher_; 
        rclcpp::Publisher<sensor_msgs::msg::Image> m_right_camera_publisher_; 
        rclcpp::Publisher<sensor_msgs::msg::Image> m_left_camera_publisher_; 

        rclcpp::TimerBase m_front_cam_timer_; 
        rclcpp::TimerBase m_right_cam_timer_; 
        rclcpp::TimerBase m_left_cam_timer_; 

        cv::VideoCapture m_front_cam_; 
        cv::VideoCapture m_right_cam_; 
        cv::VideoCapture m_left_cam_; 

		cv::Mat m_left_frame_; 
		cv::Mat m_right_frame_;
		cv::Mat m_front_frame_;

        void front_camera_callback() {
            m_front_cam_.read(m_front_frame_);

	        std_msgs::msg::Header header; 
            header.stamp = this->now();  

            msg_ = cv_bridge::CvImage(header, sensor_msgs::image_encodings::RGB8, frame).toImageMsg(); 
            m_front_camera_publisher_->publish(*msg_.get());
        } 

        void right_camera_callback() {
            right_frame_cam_.read(m_right_frame_);

	        std_msgs::msg::Header header; 
            header.stamp = this->now();  

            msg_ = cv_bridge::CvImage(header, sensor_msgs::image_encodings::RGB8, frame).toImageMsg(); 
            m_right_camera_publisher_->publish(*msg_.get());
        }

        void left_camera_callback() {
            left_camera_cam_.read(m_left_frame_);

	        std_msgs::msg::Header header; 
            header.stamp = this->now();  

            msg_ = cv_bridge::CvImage(header, sensor_msgs::image_encodings::RGB8, frame).toImageMsg(); 
            m_left_camera_publisher_->publish(*msg_.get());

        }

} 