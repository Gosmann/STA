#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <iostream>
#include <stdio.h>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include "tf2/LinearMath/Quaternion.h"
#include <tf2_ros/transform_broadcaster.h>
#include "geometry_msgs/msg/transform_stamped.hpp"

using namespace std::chrono_literals; 

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

class MinimalPublisher : public rclcpp::Node
{
  public:
    MinimalPublisher()
    : Node("minimal_publisher"), count_(0)
    {
      publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
      timer_ = this->create_wall_timer(
      500ms, std::bind(&MinimalPublisher::timer_callback, this));
    }

  private:
    void timer_callback()
    {
      auto message = std_msgs::msg::String();
      message.data = "Hello, world! " + std::to_string(count_++);
      RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
      publisher_->publish(message);
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    size_t count_;
};

class FramePublisher : public rclcpp::Node
{
public:
  FramePublisher()
  : Node("tf2_frame_publisher")
  {
    // Declare and acquire `turtlename` parameter
    //turtlename_ = this->declare_parameter<std::string>("turtlename", "turtle");

    // Initialize the transform broadcaster
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    // Subscribe to a turtle{1}{2}/pose topic and call handle_turtle_pose
    // callback function on each message
    //std::ostringstream stream;
    //stream << "/" << turtlename_.c_str() << "/pose";
    std::string topic_name = "/odom";

    subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom", 10,
      std::bind(&FramePublisher::handle_turtle_pose, this, std::placeholders::_1));
  }

private:
  void handle_turtle_pose(const std::shared_ptr<nav_msgs::msg::Odometry> msg)
  {
    geometry_msgs::msg::TransformStamped t;

    // Read message content and assign it to
    // corresponding tf variables
    t.header.stamp = this->get_clock()->now();
    t.header.frame_id = "odom";
    t.child_frame_id = "base_link";

    // Turtle only exists in 2D, thus we get x and y translation
    // coordinates from the message and set the z coordinate to 0
    t.transform.translation.x = msg->pose.pose.position.x;
    t.transform.translation.y = msg->pose.pose.position.y;
    t.transform.translation.z = 0.0;

    // For the same reason, turtle can only rotate around one axis
    // and this why we set rotation in x and y to 0 and obtain
    // rotation in z axis from the message
    tf2::Quaternion q;
    q.setRPY(0, 0, msg->pose.pose.orientation.x);
    t.transform.rotation.x = q.x();
    t.transform.rotation.y = q.y();
    t.transform.rotation.z = q.z();
    t.transform.rotation.w = q.w();

    // Send the transformation
    tf_broadcaster_->sendTransform(t);
    printf("send Transform!!! \n");
  }

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    //rclcpp::spin(std::make_shared<MinimalPublisher>());
    auto node2 = std::make_shared<FramePublisher>() ;
    

    nav_msgs::msg::Odometry msg_odom ;        
    
    auto node = rclcpp::Node::make_shared("main_robot");

    auto publisher = node->create_publisher<nav_msgs::msg::Odometry>("odom", 100);

    rclcpp::Rate loop_rate(50ms);
    int counter = 0;

    while(rclcpp::ok()){

        char buffer[100];
        sprintf(buffer, "Hello %d", counter++);

        RCLCPP_INFO(node->get_logger(), buffer);
        
        msg_odom.header.stamp = node->now();
        msg_odom.header.frame_id = "odom";

        msg_odom.child_frame_id = "base_link" ;

        msg_odom.pose.pose.position.x += 0.0001 ;

        publisher->publish(msg_odom);
        
        rclcpp::spin_some(node);
        rclcpp::spin_some(node2);

        // Initialize the transform broadcaster
        

        loop_rate.sleep();
    }


    rclcpp::shutdown();
    return 0;
}