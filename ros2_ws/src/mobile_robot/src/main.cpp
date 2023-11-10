#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <iostream>
#include <stdio.h>

#include <fcntl.h> 	// Contains file controls like O_RDWR
#include <errno.h> 	// Error integer and strerror() function
#include <termios.h> 	// Contains POSIX terminal control definitions
#include <unistd.h> 	// write(), read(), close()
#include <time.h>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include "tf2/LinearMath/Quaternion.h"
#include <tf2_ros/transform_broadcaster.h>
#include "geometry_msgs/msg/transform_stamped.hpp"

#include "/home/pi/documents/STA_github/arduino/control/pos_speed_PI_serial/control.hpp"
#include "/home/pi/documents/STA_github/arduino/control/pos_speed_PI_serial/encoders.hpp"
#include "/home/pi/documents/STA_github/arduino/control/pos_speed_PI_serial/interrupts.hpp"
#include "/home/pi/documents/STA_github/arduino/control/pos_speed_PI_serial/motors.hpp"

using namespace std::chrono_literals; 

#define PI 3.14159265478
#define MAX_BUFFER 128

#define WHEEL_D 0.062666        // diameter of the wheel in meters
#define L 0.2                   // distance between axis

typedef struct robot_t{
        
        char msg[40] ;
		encoder_t encoder_power ;
		encoder_t encoder_direc ;
			
} robot_t ;

static robot_t mobile_robot ; 

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

    static robot_t old_robot = mobile_robot;
    
    float delta_odom_m = ( mobile_robot.encoder_power.theta - old_robot.encoder_power.theta ) * WHEEL_D * 0.5 ;
    
    old_robot = mobile_robot ;
    
    static float x_robot = 0 ;
    static float y_robot = 0 ;
    static float phi_robot = 0 ;

    x_robot += ( delta_odom_m * cos( phi_robot ) ) ;
    y_robot += ( delta_odom_m * sin( phi_robot ) ) ;
    
    printf("%f \n", x_robot);    
        
    t.transform.translation.x = msg->pose.pose.position.x + x_robot  ;
    t.transform.translation.y = msg->pose.pose.position.y + y_robot  ;
    t.transform.translation.z = 0.0;

    // For the same reason, turtle can only rotate around one axis
    // and this why we set rotation in x and y to 0 and obtain
    // rotation in z axis from the message
    tf2::Quaternion q;
    static float phi = 0 ;  // pose of the robot with respect to odom
    
    phi_robot += ( delta_odom_m / L ) * tan( mobile_robot.encoder_direc.theta) ; 
    
    q.setRPY(0, 0, msg->pose.pose.orientation.x + phi_robot ) ;
    t.transform.rotation.x = q.x();
    t.transform.rotation.y = q.y();
    t.transform.rotation.z = q.z();
    t.transform.rotation.w = q.w();

    // Send the transformation
    tf_broadcaster_->sendTransform(t);
    //printf("send Transform!!! \n");
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

    rclcpp::Rate loop_rate(20ms);
    int counter = 0;
        
    // treat serial stuff
    // int serial_port = open("/dev/ttyUSB0", O_RDWR); 	// usb port
    int serial_port = open("/dev/ttyS0", O_RDWR);	// GPIO pins (Arduino mega Serial2)
    //int serial_port = open("/dev/ttyACM0", O_RDWR);	// GPIO pins (LIDAR)
    
    // serial configuration
    struct termios tty ;		    // struct that store serial configuration
    tcgetattr(serial_port, &tty) ;	
    tty.c_cc[VTIME] = 0;		    // no timeout delay	
    tty.c_cc[VMIN] = 0;		        // no timeout delay
    
    cfsetispeed(&tty, B115200); 	// define input speed
    cfsetospeed(&tty, B115200); 	// define output speed
    
    if(serial_port < 0){
	    printf("error oppening serial port \n" );
	    return -1;
    }
    else{
        //printf("Serial port oppened succesfully \n" );
    }
                   
    while(rclcpp::ok()){
        
        int i = 0 , j = 0 , n = 0 ;
       	
        char buffer[128] = {0} ;	// buffer that stores the commands    
        char buffer2[128] = {0} ;	// buffer that stores the commands    
         	
        // reads serial data
        robot_t sample;
        
        n = read(serial_port, buffer, 1);        
        
        // is new data available ?            
        if( n == 1 && buffer[j] == '$' ){   // new data is available
            //rclcpp::sleep_for(1ms);     
            //j = read(serial_port, buffer[], MAX_BUFFER);        
            
            //while( j < (sizeof(robot_t) - 1) && buffer[j] != ';' ){
            while( j < (sizeof(robot_t) - 1) ){
                j++;
                
                n = read(serial_port, &buffer[j], 1);
                
                //if( n == -1 || buffer[j] == '\n' || buffer[j] == '\0' ){
                if( n == -1 ){
                    buffer[j] = '\0' ;    
                    // print full message
                    break;
                }
            }
            //buffer[j] = '\0' ;  
            
            memcpy(&mobile_robot, &buffer, sizeof(robot_t) );
            
            //buffer[n-1] = '\0';
            sprintf(buffer2, "%d, odom : [%s] [%f] [%f] " , sizeof(robot_t), mobile_robot.msg, mobile_robot.encoder_power.theta * WHEEL_D * 0.5, mobile_robot.encoder_direc.theta ) ;  
            //sprintf(buffer, "%d, odom : [%s] " , sizeof(robot_t), mobile_robot.encoder_power.theta * 0.0666 * 0.5 ) ;  
            //printf("%d : %s \n", n, buffer) ;
            RCLCPP_INFO(node->get_logger(), buffer2);
        }
        else if(n == 0){
            //char aux[100] ;
            //rclcpp::sleep_for(10ms);               
            //read(serial_port, &sample, sizeof(robot_t) );                                
            /*
            for(i = 0 ; i < (sizeof(robot_t) - n) ; i++){
                read(serial_port, &aux, 1);                    
            } 
            *//*
            
            while( j < (MAX_BUFFER -1) ){
                j++;
                char aux;
                n = read(serial_port, &aux, 1);
                
                if( n == -1 || aux == '\n' || aux == '\0' || aux == ';' ){    
                    // print full message
                    break;
                }
            }
            */
                 
        }
        
        
   	    //close(serial_port);	// GPIO pins (Arduino mega Serial2)

        //char buffer[MAX_BUFFER];
        //sprintf(buffer, "Hello %d", counter++);


        
        msg_odom.header.stamp = node->now();
        msg_odom.header.frame_id = "odom";

        msg_odom.child_frame_id = "base_link" ;

        //msg_odom.pose.pose.position.x += 0.0001 ;
        msg_odom.pose.pose.orientation.x = 0 ;
        msg_odom.pose.pose.position.x = 0 ;

        publisher->publish(msg_odom);
        
        rclcpp::spin_some(node);
        rclcpp::spin_some(node2);

        // Initialize the transform broadcaster
        

        loop_rate.sleep();
    }


    rclcpp::shutdown();
    return 0;
}
