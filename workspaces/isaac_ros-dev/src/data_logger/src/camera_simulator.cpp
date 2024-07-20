#include <memory> 
#include <string>
#include <fstream>
#include <filesystem>
#include <ctime> 

#include <image_transport/image_transport.h> 
#include <sensor_msgs/image_encodings.hpp> 
#include <opencv2/imgproc/imgproc.hpp> 
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sys/types.h> 
#include <sys/stat.h>

#include "sensor_msgs/msg/image.hpp"
#include "rclcpp/rclcpp.hpp" 
#include "std_msgs/msg/string.hpp" 

using std::placeholders::_1; 
namespace fs = std::filesystem;

class FrameMangSub : public rclcpp::Node {
	public: 
		FrameMangSub() : Node("data_logging") : log_count(0), image_count(0){

			/* Subscribe to the front facing camera */
			subscription_front_camera = this->create_subscription<sensor_msgs::msg::Image>(
			"/video/front_camera", 10, std::bind(&FrameMangSub::topic_callback, this, _1)); 

			/* Read from left camera and check if it's open */
			left_cam.open(left_cam_id, cv::CAP_V4L2);
			if (!left_cam.isOpened()){
				RCLCPP_ERROR(this->get_logger(), "Left Camera is not open"); 
				return; 
			}

			/* Read from right camera and check if it's open */
			right_cam.open(right_cam_id, cv::CAP_V4L2);
			if (!right_cam.isOpened()){
				RCLCPP_ERROR(this->get_logger(), "Right Camera is not open"); 
				return; 
			}

			/* Create logging file if not existent */
			if (!fs::is_directory("~/logging")){
				fs::create_directory("~/logging"); 
			}

			/* Check whether directory exists or not */
			if (!fs::is_directory(drive)){
				fs::create_directory(drive);
			} else {
				/* Count the number of log files in the directory */ 
				for (const auto &entry : fs::directory_iterator(drive)) {
					log_count++; 
				} 
			} 

			/* Determine whether Image directory has been created or not */ 
			if (!fs::is_directory(image_drive)){
				fs::create_directory(image_drive); 
			} else {
				/* Count the number of images in the directory */ 
				for (const auto &entry : fs::directory_iterator(image_drive)) {
					image_count++; 
				}
			}


			/* Name the current logging file */
			logging_files = logging_files + std::to_string(log_count); 
			current_file.open(drive + "/" + logging_files + type_file); 

			/* Append the headers for file */
			current_file << "Time\t\tImage Name" << std::endl;

			/* Get current size of filesize */
			double mb = fs::file_size(drive + "/" + logging_files + type_file) / 1024 / 1024; 

			/* Check size of the tx file */
			if (mb >= 500){
				logging_files.pop_back();
				log_count++; 
				logging_files = logging_files + std::to_string(log_count);

				current_file.close(); 
				current_file.open(drive + "/" + logging_files + type_file); 
			}
			RCLCPP_DEBUG(this->get_logger(), "Finish Init"); 
		} 

		~FrameMangSub() {
			current_file.close(); 
		}


	private:
		cv::VideoCapture left_cam; 
		cv::VideoCapture right_cam; 

		const int left_cam_id = 6; 
		const int right_cam_id = 10;

		std::ofstream current_file; 

		/* '/logging' should contain logging folder */
		const std::string drive = "/logging/logging_data"; // '/' is the location of 1tb drive  
		const std::string image_drive = "/logging/image_data"; // 
		const std::string type_file = ".txt"; 

		std::string logging_files = "log_file_"; 

		int log_count;
		int image_count; 

		void topic_callback (const sensor_msgs::msg::Image &msg) {
			cv_bridge::CvImagePtr cv_ptr; 

			try {
				cv_ptr = cv_bridge::toCvCopy(msg, 
					sensor_msgs::image_encodings::BGR8); 
			} catch (cv_bridge::Exception& e) {
				RCLCPP_DEBUG(this->get_logger(), 
					"cv_bridge execption: %s", e.what()); 
				return; 
			}

			if (!current_file.is_open()){
				RCLCPP_ERROR(this->get_logger(), "File is not open"); 
				return; 
			}

			cv::Mat left_frame_cam; 
			cv::Mat right_frame_cam; 

			left_cam.read(left_frame_cam); 
			right_cam.read(right_frame_cam);

			image_count++;

			std::string front_cam_result; 
			std::string left_cam_result; 
			std::string right_cam_result;

			front_cam_result = image_drive + "/" + "image_front_" + std::to_string(image_count) + ".jpg"; 
			left_cam_result = image_drive + "/" + "image_left_" + std::to_string(image_count) + ".jpg"; 
			right_cam_result = image_drive + "/" + "image_right_" + std::to_string(image_count) + ".jpg";


			double mb = fs::file_size(drive + "/" + logging_files + type_file) / 1024 / 1024; 
		    if (mb >= 500) {
				logging_files.pop_back(); 
				log_count++; 
				logging_files = logging_files + std::to_string(log_count); 

				current_file.close(); 
				current_file.open(drive + "/" + logging_files + type_file); 	

				current_file << "Time\t\tImage Name" << std::endl;
			} 	

			/* Get the time currently */
			time_t tt; 
			std::chrono::system_clock::time_point now_time = std::chrono::system_clock::now(); 
			tt = std::chrono::system_clock::to_time_t ( now_time ); 
			
			/* Input seconds and reult to current logging file */ 
			current_file << tt << ": " << front_cam_result << " " << left_cam_result << " " << right_cam_result << std::endl; 

			/* Write to the assign directory */
			cv::imwrite(right_cam_result, right_frame_cam);
			cv::imwrite(left_cam_result, left_frame_cam);
			cv::imwrite(front_cam_result, cv_ptr->image); 

			RCLCPP_INFO(this->get_logger(), "%s", front_cam_result.c_str()); 
		} 

		 
		rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_front_camera; 
}; 

int main(int argc, char *argv[]){
	rclcpp::init(argc, argv); 
       	rclcpp::spin(std::make_shared<FrameMangSub>()); 
	rclcpp::shutdown(); 

	return 0; 
} 	
