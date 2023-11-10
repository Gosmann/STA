#include <functional>
#include <memory>
#include <string>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include "nav_msgs/msg/occupancy_grid.hpp"

#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include <tf2_ros/transform_broadcaster.h>
#include "geometry_msgs/msg/transform_stamped.hpp"

#define RANGE 6 

using namespace std::chrono_literals;
using std::placeholders::_1;

typedef struct pose_t {
    float x;
    float y; 
    float theta;
} pose_t ;

pose_t robot ;
int * data ;
int width;
int heigth;


//ros::Publisher map_pub;

/*
void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg){
  std_msgs::Header header = msg->header;
  nav_msgs::MapMetaData info = msg->info;
  ROS_INFO("Got map %d %d", info.width, info.height);
  Map map(info.width, info.height);
  for (unsigned int x = 0; x < info.width; x++)
    for (unsigned int y = 0; y < info.height; y++)
      map.Insert(Cell(x,y,info.width,msg->data[x+ info.width * y]));
  nav_msgs::OccupancyGrid* newGrid = map.Grid();
  newGrid->header = header;
  newGrid->info = info;
  map_pub.publish(*newGrid);
}
*/

//using namespace std::chrono_literals; 

class MapSubscriber : public rclcpp::Node
{
public:
    MapSubscriber()
    : Node("map_subscriber")
    {
    subscription_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
      "map", 10, std::bind(&MapSubscriber::topic_callback, this, _1));
      
    //subscription_pose = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    //  "pose",10,std::bind(&MapSubscriber::topic_callback,this,_1));

    }

private:
  //void topic_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr & msg,const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr & RobotPos) const
  void topic_callback( const nav_msgs::msg::OccupancyGrid::SharedPtr msg) 
  {
    // RCLCPP_INFO(this->get_logger(), "I heard: '%d'", msg->info.width);
    // RCLCPP_INFO(this->get_logger(), "I heard: '%d'", RobotPos->pose.pose.position.x);
    printf("Get callback! \n");
    width = msg->info.width ;
    heigth = msg->info.height;
    
    //robot.x= RobotPos->pose.pose.position.x ;
    //robot.y= RobotPos->pose.pose.position.y ;
    //robot.theta= RobotPos->pose.pose.orientation.w ;
  }
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr subscription_;
  //rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr subscription_pose;
};



int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  
  //rclcpp::spin(std::make_shared<MapSubscriber>());
 rclcpp::Rate loop_rate(100ms);
 auto node_sub = std::make_shared<MapSubscriber>() ;
  
   
  while(1){
    rclcpp::spin_some(node_sub);
    printf("heigth : %d, width : %d \n", heigth, width);
    
    loop_rate.sleep();
  }
  
  
  return 0;
}

