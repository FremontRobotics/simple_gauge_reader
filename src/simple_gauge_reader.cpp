#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
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
 
/*class GaugeReaderServer : public rclcpp::Node {
public:
  GaugeReaderServer() : Node("opencv_image_publisher"), count_(0) {
    publisher_ =
        this->create_publisher<sensor_msgs::msg::Image>("random_image", 10);
    timer_ = this->create_wall_timer(
        5s, std::bind(&GaugeReaderServer::timer_callback, this));
  }
 
private:
  void timer_callback() {
    // Create a new 640x480 image
    cv::Mat my_image(cv::Size(640, 480), CV_8UC3);
 
    // Generate an image where each pixel is a random color
    cv::randu(my_image, cv::Scalar(0, 0, 0), cv::Scalar(255, 255, 255));
 
    // Write message to be sent. Member function toImageMsg() converts a CvImage
    // into a ROS image message
    cv_bridge::CvImage cv_image = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", my_image);
    msg_ = cv_image.toImageMsg();

    // Publish the image to the topic defined in the publisher
    publisher_->publish(*msg_.get());
    RCLCPP_INFO(this->get_logger(), "Image %ld published", count_);

    cv::imshow("stuff", cv_image.image);
    cv::waitKey();
    count_++;
  }
  rclcpp::TimerBase::SharedPtr timer_;
  sensor_msgs::msg::Image::SharedPtr msg_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
  size_t count_;

};*/

void read_gauge(const std::shared_ptr<simple_gauge_reader_interfaces::srv::ReadGauge::Request> request,
                std::shared_ptr<simple_gauge_reader_interfaces::srv::ReadGauge::Response>      response)
{
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Incoming request\na: %ld" " b: %ld",
                request->center_x, request->center_y);

    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(request->img, request->img.encoding);
    }
    catch (cv_bridge::Exception& e)
    {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "cv_bridge exception: %s", e.what());
        return;
    }


    cv::Mat gray_mat = cv::Mat(cv_ptr->image.rows, cv_ptr->image.cols, CV_8UC1);
    cvtColor(cv_ptr->image, gray_mat, cv::COLOR_BGR2GRAY, 0);
    
    Gauge *gauge = new Gauge(gray_mat, 
                              cv::Point(request->center_x, request->center_y),
                              cv::Point(request->min_x, request->min_y),
                              cv::Point(request->max_x, request->max_y));

    
    double value = gauge->ComputeGaugeValue(gray_mat);
    response->value = value;

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "sending back response: [%f]", response->value);

}
 
int main(int argc, char *argv[]) {

  rclcpp::init(argc, argv);

  // create a ros2 node
  auto node = rclcpp::Node::make_shared("simple_gauge_reader_server");

  rclcpp::Service<simple_gauge_reader_interfaces::srv::ReadGauge>::SharedPtr service = 
    node->create_service<simple_gauge_reader_interfaces::srv::ReadGauge>("simple_gauge_reader", &read_gauge);

  RCLCPP_INFO(rclcpp::get_logger("rclpp"), "Ready for ReadGauge requests.");
 
  // process ros2 callbacks until receiving a SIGINT (ctrl-c)
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
