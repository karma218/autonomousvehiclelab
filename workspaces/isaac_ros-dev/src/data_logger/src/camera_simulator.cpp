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
		FrameMangSub() : Node("frame_subscriber"){
			subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
			"image_stitching", 10, std::bind(&FrameMangSub::topic_callback, this, _1)); 

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
		std::ofstream current_file; 

		/* '/logging' should contain logging folder */
		const std::string drive = "/media/snow/Heavy_Duty/logging_data"; // '/' is the location of 1tb drive  
		const std::string image_drive = "/media/snow/Heavy_Duty/image_data"; // 

		std::string logging_files = "log_file_"; 
		std::string type_file = ".txt"; 

		int log_count = 0; 
		int image_count = 0; 

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

			image_count++;
			std::string result; 
			result = image_drive + "/" + "image_" + std::to_string(image_count) + ".jpg"; 

			double mb = fs::file_size(drive + "/" + logging_files + type_file) / 1024 / 1024; 
		       	if (mb >= 500) {
				logging_files.pop_back(); 
				log_count++; 
				logging_files = logging_files + std::to_string(log_count); 

				current_file.close(); 
				current_file.open(drive + "/" + logging_files + type_file); 	

				current_file << "Time\t\tImage Name" << std::endl;
			} 	


			time_t tt; 
			std::chrono::system_clock::time_point now_time = std::chrono::system_clock::now(); 
			tt = std::chrono::system_clock::to_time_t ( now_time ); 
			
			/* Input seconds and reult to current logging file */ 
			current_file << tt << ": " << result << std::endl; 

			cv::imwrite(result, cv_ptr->image); 
			RCLCPP_INFO(this->get_logger(), "%s", result.c_str()); 
		} 

		 
		rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_; 
}; 

int main(int argc, char *argv[]){
	rclcpp::init(argc, argv); 
       	rclcpp::spin(std::make_shared<FrameMangSub>()); 
	rclcpp::shutdown(); 

	return 0; 
} 	
