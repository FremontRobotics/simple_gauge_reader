#include "rclcpp/rclcpp.hpp"
#include "rclcpp/subscription_base.hpp"
#include "rclcpp/subscription_options.hpp"
#include "rclcpp/utilities.hpp"
#include "rclcpp/wait_for_message.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "simple_gauge_reader_interfaces/srv/read_gauge.hpp"

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
class SimpleGaugeReader;

class SimpleGaugeReader : public rclcpp::Node {

public:
  SimpleGaugeReader() : Node("simple_gauge_reader") {
    callback_group_1_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    callback_group_2_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    
    start_image_sub("/j100_0803/axis_ptz/image_raw/compressed");
    
    srv_ = create_service<simple_gauge_reader_interfaces::srv::ReadGauge>("simple_gauge_reader", 
                                                                          std::bind(&SimpleGaugeReader::read_gauge, this, _1, _2),
                                                                          rmw_qos_profile_services_default, 
                                                                          callback_group_1_);




    RCLCPP_INFO(rclcpp::get_logger("rclpp"), "Ready for ReadGauge requests.");
  
  }

private:
  rclcpp::Service<simple_gauge_reader_interfaces::srv::ReadGauge>::SharedPtr srv_;
  rclcpp::CallbackGroup::SharedPtr callback_group_1_;
  rclcpp::CallbackGroup::SharedPtr callback_group_2_;
  rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr subscription_;
  sensor_msgs::msg::CompressedImage image_msg_;
  std::shared_ptr<simple_gauge_reader_interfaces::srv::ReadGauge::Request> request_;
  bool image_found = false;


  void read_gauge(const std::shared_ptr<simple_gauge_reader_interfaces::srv::ReadGauge::Request> request,
                  std::shared_ptr<simple_gauge_reader_interfaces::srv::ReadGauge::Response>      response)
  {
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Incoming request\na: %ld" " b: %ld",
                  request->center_x, request->center_y);

      //get one message from topic

      RCLCPP_INFO(this->get_logger(), "starting to wait for image message");

      while (!image_found) {
        RCLCPP_INFO(this->get_logger(), "still waiting for image");
        rclcpp::sleep_for(1s);
      }
      RCLCPP_INFO(this->get_logger(), "message found");

      
      cv_bridge::CvImagePtr cv_ptr;
      try
      {

          cv_ptr = cv_bridge::toCvCopy(image_msg_, "bgr8");
      }
      catch (cv_bridge::Exception& e)
      {
          RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "cv_bridge exception: %s", e.what());
          return;
      }

      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "finished toCvCopy");

      cv::Mat gray_mat = cv::Mat(cv_ptr->image.rows, cv_ptr->image.cols, CV_8UC1);

      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "created Mat");

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

      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "sending back response: [%f]", response->value);

  }

  void start_image_sub(std::string topic) {

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "subscribing to topic [%s]", topic.c_str());
    rclcpp::SubscriptionOptions sub_options;
    sub_options.callback_group = callback_group_2_;
    subscription_ = this->create_subscription<sensor_msgs::msg::CompressedImage>(
        topic, 10,
        std::bind(&SimpleGaugeReader::image_callback, this, _1),
        sub_options);
  }

  void image_callback(const sensor_msgs::msg::CompressedImage & msg) {
    image_msg_ = msg;
    image_found = true;
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "received");
    
  }
};

int main(int argc, char *argv[]) {

  rclcpp::init(argc, argv);

  // create a ros2 node
  auto node = std::make_shared<SimpleGaugeReader>();

  // process ros2 callbacks until receiving a SIGINT (ctrl-c)
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
