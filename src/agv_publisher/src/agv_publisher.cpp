#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <ctime>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

class AGVPublisher : public rclcpp::Node
{
  public:
    AGVPublisher()
    : Node("agv_publisher"), count_(0)
    {
      publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/tb3_1/cmd_vel", 10);
      timer_ = this->create_wall_timer(
      100ms, std::bind(&AGVPublisher::timer_callback, this));
    }

  private:
    void timer_callback()
    {
      if (count_ == 400) {
        auto message = geometry_msgs::msg::Twist();
        message.linear.x = .2;
        message.linear.y = 0;
        message.linear.z = 0;
        message.angular.x = 0;
        message.angular.y = 0;
        message.angular.z = 0;

        publisher_->publish(message);
        count_++;
      }
      else if (count_ == 800) {
        auto message = geometry_msgs::msg::Twist();
        message.linear.x = -.2;
        message.linear.y = 0;
        message.linear.z = 0;
        message.angular.x = 0;
        message.angular.y = 0;
        message.angular.z = 0;

        publisher_->publish(message);
        count_ = 0;
      }
      else {
        count_++;
      }
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    size_t count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AGVPublisher>());
  rclcpp::shutdown();
  return 0;
}

