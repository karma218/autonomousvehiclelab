#include <memory> 
#include <string>
#include <fstream>
#include <filesystem>
#include <ctime> 
#include <mutex>

#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <sys/types.h> 
#include <sys/stat.h>
#include <message_filters/subscriber.h> 
#include <message_filters/time_synchronizer.h>

#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/image_encodings.hpp" 
#include "image_transport/image_transport.hpp" 
#include "sensor_msgs/msg/image.hpp"
#include "rclcpp/rclcpp.hpp" 
#include "std_msgs/msg/string.hpp" 

using std::placeholders::_1; 
namespace fs = std::filesystem;

class DataLogger : public rclcpp::Node {
	public: 
		DataLogger() : Node("data_logger_node"), 
            m_log_count(0), m_image_count(0), {

			/* Create logging file if not existent */
			if (!fs::is_directory("/home/admin/logging")){
                RCLCPP_INFO(this->get_logger(), "Creating logging directory");

				fs::create_directory("/home/admin/logging"); 
                assert(!std::fs::create_directory("/home/admin/logging"));
			}

			/* Check whether directory exists or not */
			if (!fs::is_directory(m_drive)){
                RCLCPP_INFO(this->get_logger(), "Creating logging files directory");

				fs::create_directory(m_drive);
                assert(!std::fs::create_directory(m_drive));
			} else {
				/* TODO: Use std::distance or std::count_if */
				/* Count the number of log files in the directory */ 
				for (const auto &entry : fs::directory_iterator(m_drive)) {
					m_log_count++; 
				} 
			} 

            assert(!std::fs::create_directory(m_drive));

			/* Determine whether Image directory has been created or not */ 
			if (!fs::is_directory(m_image_drive)){
                RCLCPP_INFO(this->get_logger(), "Creating Image directory");

				fs::create_directory(m_image_drive); 
                assert(!std::fs::create_directory(m_image_drive));
			} else {
				/* TODO: Use std::distance or std::count_if */
				/* Count the number of images in the directory */ 
				for (const auto &entry : fs::directory_iterator(m_image_drive)) {
					m_image_count++; 
				}
			}


			m_logging_files = m_logging_files + std::to_string(m_log_count); 
			m_current_file.open(m_drive + "/" + m_logging_files + m_type_file); 

			/* Append the headers for file */
			m_current_file << "Time\t\tImage Name\t\tSteering" << std::endl;

			/* Get current size of filesize */
			double mb = fs::file_size(m_drive + "/" + m_logging_files + m_type_file) / 1024 / 1024; 

			/* Check size of the tx file */
			if (mb >= 50){
				m_logging_files.pop_back();
				m_log_count++; 
				m_logging_files = m_logging_files + std::to_string(m_log_count);

				m_current_file.close(); 
				m_current_file.open(m_drive + "/" + m_logging_files + m_type_file); 
			}

            front_camera.subscribe(this, "/video/font_fisheye_camera"); 
            left_camera.subscribe(this, "/video/left_camera"); 
            right_camera.subscribe(this, "/video/right_camera"); 
            steering_msg.subscribe(this, "/twist_mux/cmd_vel");

            typedef message_filters::sync_policies::ApproximateTimer<sensor_msgs::msg::Image, sensor_msgs::msgs::Image, 
                sensor_msgs::msgs::Image, geometry_msgs::msgs::Twist> approximate_policy;

            message_filters::Synchronizer<approximate_policy>syncApproximate(
                approximate_policy(1, 0), front_camera, left_camera, right_camera, steering_msg); 

            syncApproximate.setMaxIntervalDuration(rclcpp::Duration(3, 0));
            syncApproximate.registerCallback(&DataLogger::camera_sync_callback, this);

			RCLCPP_DEBUG(this->get_logger(), "Finish Init"); 
        }

        ~DataLogger() {
			m_left_cam.release(); 
			m_right_cam.release();

			m_current_file.close(); 
        }

    private: 
        /* Subscribers to cameras and steering*/
        message_filters::Subscriber<sensor_msgs::msg::Image> m_front_camera; 
        message_filters::Subscriber<sensor_msgs::msg::Image> m_left_camera;
        message_filters::Subscriber<sensor_msgs::msg::Image> m_right_camera;
        message_filters::Subscriber<geometry_msgs::msg::Twist> m_steering_msg;

        /* File handler */
		std::ofstream m_current_file; 

		/* '/logging' should contain logging folder */
		const std::string m_drive = "/home/admin/logging/logging_data"; // '/' is the location of 1tb drive  
		const std::string m_image_drive = "/home/admin/logging/image_data"; // 
		const std::string m_type_file = ".txt"; 

		std::string m_logging_files = "log_file_"; 

		int m_log_count;
		int m_image_count; 
        void camera_sync_callback(const sensor_msgs::msg::Image &front_image, const sensor_msgs::msg::Image &left_image, 
            const sensor_msgs::msg::Image &right_image, const geometry_msgs::msg::Twist &msg){

            if (!m_current_file.is_open()){
                RCLCPP_ERROR(this->get_logger, "Logging file isn't opened for logging");
                return;
            }
        
            /* Process results from all the cameras */
            cv_bridge::CvImagePtr cv_front_ptr; 
            cv_bridge::CvImagePtr cv_left_ptr; 
            cv_bridge::CvImagePtr cv_right_ptr; 
            
            cv_front_ptr = cv_bridge::toCvCopy(front_image, sensor_msgs::image_encodings::BGR8);
            cv_left_ptr = cv_bridge::toCvCopy(left_image, sensor_msgs::image_encodings::BGR8);
            cv_right_ptr = cv_bridge::toCvCopy(right_camera, sensor_msgs::image_encodings::BGR8);

            /* Prepare the images name */
			std::string front_cam_result; 
			std::string left_cam_result; 
			std::string right_cam_result;

			front_cam_result = m_image_drive + "/" + "image_front_" + std::to_string(m_image_count) + ".jpg"; 
			left_cam_result = m_image_drive + "/" + "image_left_" + std::to_string(m_image_count) + ".jpg"; 
			right_cam_result = m_image_drive + "/" + "image_right_" + std::to_string(m_image_count) + ".jpg";

            /* Calculate the current steering angle */
			double steering_angle = msg->angular.z;
            int steering_angle_int = steering_angle * 61.0 + 512.0;

            /* Ensure log file is below 50 MB */
			double mb = fs::file_size(m_drive + "/" + m_logging_files + m_type_file) / 1024 / 1024; 
		    if (mb >= 50) {
				m_logging_files.pop_back(); 
				m_log_count++; 
				m_logging_files = m_logging_files + std::to_string(m_log_count); 

				m_current_file.close(); 
				m_current_file.open(m_drive + "/" + m_logging_files + m_type_file); 	

				m_current_file << "Time\t\tImage Name\t\tSteering Value" << std::endl;
			} 	

            /* Get the current time to set to file */
			time_t tt; 
			std::chrono::system_clock::time_point now_time = std::chrono::system_clock::now(); 
			tt = std::chrono::system_clock::to_time_t ( now_time ); 
			
			/* Input seconds and reult to current logging file */ 
			m_current_file << tt << ": " << front_cam_result << " " << left_cam_result << " " << right_cam_result <<  " " << steering_angle_int << std::endl; 

			/* Write to the assign directory */
			if (!cv::imwrite(right_cam_result, cv_front_ptr->image)){
                RCLCPP_DEBUG(this->get_logger(), "Failed to write right camera frame");
                return;
            }

			if (!cv::imwrite(left_cam_result, cv_left_ptr->image)){
                RCLCPP_DEBUG(this->get_logger(), "Failed to write left camera frame"); 
                return;
            }

			if (!cv::imwrite(front_cam_result, cv_right_ptr->image)){
                RCLCPP_DEBUG(this->get_logger(), "Failed to write front camera frame"); 
                return;
            }; 

			RCLCPP_INFO(this->get_logger(), "%s", front_cam_result.c_str()); 

        }

}

int main(int argc, char* argv[]){
	rclcpp::init(argc, argv); 
    rclcpp::spin();
	rclcpp::shutdown(); 

	return 0; 
} 	