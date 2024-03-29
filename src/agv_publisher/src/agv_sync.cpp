#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <ctime>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

class AGVSync : public rclcpp::Node
{
  public:
    AGVSync()
    : Node("agv_sync")
    {
      publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/goal_pose", 10);
      timer_ = this->create_wall_timer(
      100ms, std::bind(&AGVSync::timer_callback, this));
      publish_spot_ = 1; // initialize to 0 meaning no publish
      target_frame_ = this->declare_parameter<std::string>("target_frame", "map");
      tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
      tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    }

  private:
     void get_pose(geometry_msgs::msg::TransformStamped &t) {
        std::string fromFrameRel = "map";
        std::string toFrameRel = "tb3_1/base_link";

        // Look up for the transformation between target_frame and turtle2 frames
        // and send velocity commands for turtle2 to reach target_frame
        try {
          t = tf_buffer_->lookupTransform(
            toFrameRel, fromFrameRel,
            tf2::TimePointZero);
        } catch (const tf2::TransformException & ex) {
          RCLCPP_INFO(
            this->get_logger(), "Could not transform %s to %s: %s",
            toFrameRel.c_str(), fromFrameRel.c_str(), ex.what());
          return;
        }
        
        return;
    }
    void timer_callback()
    {
      if (publish_spot_ == 1) {
        auto message = geometry_msgs::msg::PoseStamped();
        message.pose.position.x = 1.5;
        message.pose.position.y = -1.5;
        message.pose.position.z = 0;
        message.pose.orientation.x = 0;
        message.pose.orientation.y = 0;
        message.pose.orientation.z = 0;
        message.pose.orientation.w = 1;
        message.header.stamp = now();
        message.header.frame_id = "map";

        publisher_->publish(message);

        // publish_spot_ = 0;
      }
      else if (publish_spot_ == 2) {
        geometry_msgs::msg::TransformStamped t;
        get_pose(t);
        auto message = geometry_msgs::msg::PoseStamped();
        message.pose.position.x = t.transform.translation.x;
        message.pose.position.y = t.transform.translation.y;
        message.pose.position.z = t.transform.translation.z;
        message.pose.orientation.x = t.transform.rotation.x;
        message.pose.orientation.y = t.transform.rotation.y;
        message.pose.orientation.z = t.transform.rotation.z;
        message.pose.orientation.w = t.transform.rotation.w;
        message.header.stamp = now();
        message.header.frame_id = "map";


        publisher_->publish(message);
      }
      else {
      }
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher_;
    int publish_spot_;
    std::string target_frame_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AGVSync>());
  rclcpp::shutdown();
  return 0;
}

