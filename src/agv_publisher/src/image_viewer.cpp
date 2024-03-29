#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <ctime>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/Image.hpp"
using std::placeholders::_1;

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

class ViewImage : public rclcpp::Node
{
  public:
    ViewImage()
    : Node("view_image"), count_(0)
    {
      subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
      "/camera1/image_raw", 10, std::bind(&ViewImage::topic_callback, this, _1));
    }

  private:
    void topic_callback(const sensor_msgs::msg::Image::SharedPtr msg) const
    {
      auto data = msg->data;
      int width = msg->width;
      int height = msg->height;
      int step = msg->step;

      vector<int> 
      for (int i = 0; i < width*height; i++)
      {

      }
      
    }
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ViewImage>());
  rclcpp::shutdown();
  return 0;
}

