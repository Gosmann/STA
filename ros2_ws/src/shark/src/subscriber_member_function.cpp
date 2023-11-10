#include <functional>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"

#define RANGE 6 

using std::placeholders::_1 ;

typedef struct Robot{
    float x;
    float y; 
    float theta;
};

struct Robot Ex ;
int * data ;
int width;
int heigth;


class MapSubscriber : public rclcpp::Node
{
public:
  MapSubscriber()
  : Node("map_subscriber")
  {
    subscription_map = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
      "map", 10, std::bind(&MapSubscriber::topic_callback, this, _1));
    subscription_pose = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "pose",10,std::bind(&MapSubscriber::topic_callback,this,_1));

  }

private:
  void topic_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr & msg,const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr & RobotPos) const
  {
    // RCLCPP_INFO(this->get_logger(), "I heard: '%d'", msg->info.width);
    // RCLCPP_INFO(this->get_logger(), "I heard: '%d'", RobotPos->pose.pose.position.x);
    width=msg->info.width ;
    heigth=msg->info.height;

    Ex.x= RobotPos->pose.pose.position.x ;
    Ex.y= RobotPos->pose.pose.position.y ;
    Ex.theta= RobotPos->pose.pose.orientation.w ;
  }
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr subscription_map;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr subscription_pose;
};

int Inconnu(int * tableau,int i){
  int n;
  int c = 0;
  int size = width*heigth ;
  int line = i/width;
  int k;
  //1 droite
  for(n=1;n<=RANGE;n++){
    k=i+n;
    if(k>=0 and k<=size and k/width==line){
      if (tableau[k]==-1){c++;}
      else{if(tableau[k]==100){break;}}}
    else{break;}
  }
  //2 gauche
  for(n=1;n<=RANGE;n++){
    k=i-n;
    if(k>=0 and k<=size and k/width==line){
      if (tableau[k]==-1){c++;}
      else{if(tableau[k]==100){break;}}}
    else{break;}
  }
  //3 haut
  for(n=1;n<=RANGE;n++){
    k=i+n*width;
    if(k>=0 and k<=size){
      if (tableau[k]==-1){c++;}
      else{if(tableau[k]==100){break;}}}
    else{break;}
  }
  //4 bas
  for(n=1;n<=RANGE;n++){
    k=i-n*width ;
    if(k>=0 and k<=size){
      if (tableau[k]==-1){c++;}
      else{if(tableau[k]==100){break;}}}
    else{break;}
  }
  //5 bas droite
  for(n=1;n<=RANGE;n++){
    k=i+n+n*width;
    if(k>=0 and k<=size and (i+n)/width==line){
      if (tableau[k]==-1){c++;}
      else{if(tableau[k]==100){break;}}}
    else{break;}
  }
  //6 bas gauche
  for(n=1;n<=RANGE;n++){
    k=i-n+n*width ;
    if(k>=0 and k<=size and (i-n)/width==line){
      if (tableau[k]==-1){c++;}
      else{if(tableau[k]==100){break;}}}
    else{break;}
  }
  //7 haut gauche
  for(n=1;n<=RANGE;n++){
    k=i-n-n*width ;
    if(k>=0 and k<=size and (i-n)/width==line){
      if (tableau[k]==-1){c++;}
      else{if(tableau[k]==100){break;}}}
    else{break;}
  }
  //8 haut droite
  for(n=1;n<=RANGE;n++){
    k=i+n-n*width ;
    if(k>=0 and k<=size and (i+n)/width==line){
      if (tableau[k]==-1){c++;}
      else{if(tableau[k]==100){break;}}}
    else{break;}
  }

  return c ; 
}

int Obstacle(int * tableau,int i){
  //fonction calcule le "poids" des obbstacles autour de la position i
  int n;
  int c = 0;
  int size = width*heigth ;
  int line = i/width;
  int k;
  //1 droite
  for(n=1;n<=RANGE;n++){
    k=i+n;
    if(k>=0 and k<=size and k/width==line){
      if(tableau[k]==100){
        c=RANGE-n;
        break;}}
    else{break;}
  }
  //2 gauche
  for(n=1;n<=RANGE;n++){
    k=i-n;
    if(k>=0 and k<=size and k/width==line){
      if(tableau[k]==100){
        c=RANGE-n;
        break;}}
    else{break;}
  }
  //3 haut
  for(n=1;n<=RANGE;n++){
    k=i+n*width;
    if(k>=0 and k<=size){
      if(tableau[k]==100){
        c=RANGE-n;
        break;}}
    else{break;}
  }
  //4 bas
  for(n=1;n<=RANGE;n++){
    k=i-n*width ;
    if(k>=0 and k<=size){
      if(tableau[k]==100){
        c=RANGE-n;
        break;}}
    else{break;}
  }
  //5 bas droite
  for(n=1;n<=RANGE;n++){
    k=i+n+n*width;
    if(k>=0 and k<=size and (i+n)/width==line){
      if(tableau[k]==100){
        c=RANGE-n;
        break;}}
    else{break;}
  }
  //6 bas gauche
  for(n=1;n<=RANGE;n++){
    k=i-n+n*width ;
    if(k>=0 and k<=size and (i-n)/width==line){
      if(tableau[k]==100){
        c=RANGE-n;
        break;}}
    else{break;}
  }
  //7 haut gauche
  for(n=1;n<=RANGE;n++){
    k=i-n-n*width ;
    if(k>=0 and k<=size and (i-n)/width==line){
      if(tableau[k]==100){
        c=RANGE-n;
        break;}}
    else{break;}
  }
  //8 haut droite
  for(n=1;n<=RANGE;n++){
    k=i+n-n*width ;
    if(k>=0 and k<=size and (i+n)/width==line){
      if(tableau[k]==100){
        c=RANGE-n;
        break;}}
    else{break;}
  }

  return c; 
}



int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MapSubscriber>());
  
  rclcpp::shutdown();
  return 0;
}