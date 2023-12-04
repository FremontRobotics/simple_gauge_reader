#include "rclcpp/rclcpp.hpp"
#include "simple_gauge_reader_interfaces/srv/read_gauge_from_compressed_image.hpp"

#include "std_msgs/msg/header.hpp"
#include <chrono>
#include <cv_bridge/cv_bridge.h> // cv_bridge converts between ROS 2 image messages and OpenCV image representations.
#include <image_transport/image_transport.hpp> // Using image_transport allows us to publish and subscribe to compressed image streams in ROS2
#include <opencv2/opencv.hpp> // We include everything about OpenCV as we don't care much about compilation time at the moment.
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "gauge.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;

class SimpleGaugeReaderFromCompressedImage : public rclcpp::Node {

public:
  SimpleGaugeReaderFromCompressedImage() : Node("simple_gauge_reader_from_compressed_image") {
    callback_group_1_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    //callback_group_2_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    
   
    
    srv_ = create_service<simple_gauge_reader_interfaces::srv::ReadGaugeFromCompressedImage>("simple_gauge_reader_from_compressed_image", 
                                                                          std::bind(&SimpleGaugeReaderFromCompressedImage::read_gauge_from_compressed_image, this, _1, _2),
                                                                          rmw_qos_profile_services_default, 
                                                                          callback_group_1_);




    RCLCPP_INFO(rclcpp::get_logger("rclpp"), "Ready for ReadGaugeFromCompressedImage requests.");
  
  }

private:
  rclcpp::Service<simple_gauge_reader_interfaces::srv::ReadGaugeFromCompressedImage>::SharedPtr srv_;
  rclcpp::CallbackGroup::SharedPtr callback_group_1_;
  //rclcpp::CallbackGroup::SharedPtr callback_group_2_;
  rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr subscription_;
  sensor_msgs::msg::CompressedImage image_msg_;
  std::shared_ptr<simple_gauge_reader_interfaces::srv::ReadGaugeFromCompressedImage::Request> request_;
 // bool image_found = false;


  void read_gauge_from_compressed_image(const std::shared_ptr<simple_gauge_reader_interfaces::srv::ReadGaugeFromCompressedImage::Request> request,
                  std::shared_ptr<simple_gauge_reader_interfaces::srv::ReadGaugeFromCompressedImage::Response>      response)
  {
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Incoming request - coords: %ld,%ld %ld,%ld %ld,%ld",
                request->center_x, request->center_y, request->min_x, request->min_y,request->max_x, request->max_y);

      
      cv_bridge::CvImagePtr cv_ptr;
      try
      {   
          cv_ptr = cv_bridge::toCvCopy(request->img, sensor_msgs::image_encodings::BGR8);
      }
      catch (cv_bridge::Exception& e)
      {
          RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "cv_bridge exception: %s", e.what());
          return;
      }

      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "finished toCvCopy");

      cv::Mat gray_mat = cv::Mat(cv_ptr->image.rows, cv_ptr->image.cols, CV_8UC1);

      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "created Mat: %d x %d", cv_ptr->image.rows, cv_ptr->image.cols);

      cvtColor(cv_ptr->image, gray_mat, cv::COLOR_BGR2GRAY, 0);
      

      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "finished cvtColor");

      Gauge *gauge = new Gauge(gray_mat, 
                                cv::Point(request->center_x, request->center_y),
                                cv::Point(request->min_x, request->min_y),
                                cv::Point(request->max_x, request->max_y));

      
      double value;
      try 
      {
        value = gauge->ComputeGaugeValue(gray_mat);
        response->value = value;
      }
      catch (cv::Exception& e) 
      {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "c exception: %s", e.what());
        value = -9999.99;
      }

      int range = request->range_max - request->range_min;
      int scaled = range * (response->value / 100);
      int newResult = request->range_min + scaled;
      response->value = newResult;

      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "sending back response: [%f]", response->value);

  }

};

int main(int argc, char *argv[]) {

  rclcpp::init(argc, argv);

  // create a ros2 node
  auto node = std::make_shared<SimpleGaugeReaderFromCompressedImage>();

  // process ros2 callbacks until receiving a SIGINT (ctrl-c)
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
