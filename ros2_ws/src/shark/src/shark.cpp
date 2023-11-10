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

// path planning
// #include "display.h"
#include "kinematics.h"

#define RANGE 6 

using namespace std::chrono_literals;
using std::placeholders::_1;

int flag_new_pose = 0;
int flag_new_map = 0;

pose_t robot_slam = {0} ;
pose_t origin = {0} ;
int8_t * map_now ;

int width = 0;
int heigth = 0;
float resolution = 0;


//ros::Publisher map_pub;



//using namespace std::chrono_literals; 

class MapSubscriber : public rclcpp::Node
{
public:
    MapSubscriber()
    : Node("map_subscriber")
    {
    subscription_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
      "map", 10, std::bind(&MapSubscriber::topic_callback, this, _1));
      
    subscription_pose = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "pose", 10, std::bind(&MapSubscriber::topic_callback_pose, this,_1));

    }

private:
  //void topic_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr & msg,const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr & RobotPos) const
  void topic_callback( const nav_msgs::msg::OccupancyGrid::SharedPtr msg) 
  {
	  flag_new_map = 1;
    // RCLCPP_INFO(this->get_logger(), "I heard: '%d'", msg->info.width);
    // RCLCPP_INFO(this->get_logger(), "I heard: '%d'", RobotPos->pose.pose.position.x);
    printf("Get callback! \n");
    width = msg->info.width ;
    heigth = msg->info.height;
    resolution = msg->info.resolution ;
    
    origin.x = (float) msg->info.origin.position.x ;
    origin.y = (float) msg->info.origin.position.y ;
    
	map_now = (int8_t *)malloc( width * heigth ) ;
	
	int i = 0;
	for( i = 0 ; i < width * heigth ; i++){
		map_now[i] = msg->data[i] ;
	}
	
	//memcpy(map_now, msg->data, width * heigth) ;
    
    
    //robot.x= RobotPos->pose.pose.position.x ;
    //robot.y= RobotPos->pose.pose.position.y ;
    //robot.theta= RobotPos->pose.pose.orientation.w ;
  }
  
  void topic_callback_pose( const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) 
  {
	  flag_new_pose = 1;
	  printf(" POSE!!! \n");
    // RCLCPP_INFO(this->get_logger(), "I heard: '%d'", msg->info.width);
    // RCLCPP_INFO(this->get_logger(), "I heard: '%d'", RobotPos->pose.pose.position.x);
        
    robot_slam.x= msg->pose.pose.position.x ;
    robot_slam.y= msg->pose.pose.position.y ;
    robot_slam.theta= msg->pose.pose.orientation.x ;
    
    
  }
  
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr subscription_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr subscription_pose;
};


node_t * best_node ;

void expand_node( node_t * node, double dt ){
    
    uint8_t actions[6] = { 'L', 'R', 'F', 'B', 'Z', 'X' } ;

    int i;
    for(i = 0 ; i < CHILDREN_NUM ; i++){
        
        node_t copy_node = node[0];
        
        switch( actions[i] ){
            case Left_Forwards:                    // expand left forwards
                copy_node.state.phi = 0.10 ;       // 5.7º
                copy_node.state.speed = 0.08 ;     // move forwards at 0.08 m/s    
                node->children[ i ] = simulate( &copy_node, dt, Left_Forwards);
                break;
            case Right_Forwards:                   // expand right forwards
                copy_node.state.phi = -0.10 ;      // -15º
                copy_node.state.speed = 1 ;        // move forwards at 1 m/s    
                node->children[ i ] = simulate( &copy_node, dt, Right_Forwards);
                break;
            case Forwards:                         // expand forwards
                copy_node.state.phi = 0 ;          // 0º
                copy_node.state.speed = 1 ;        // move forwards at 1 m/s    
                node->children[ i ] = simulate( &copy_node, dt, Forwards);
                break;
            case Backwards:                        // expand backwards
                copy_node.state.phi = 0 ;          // 0º
                copy_node.state.speed = -1 ;       // move forwards at 1 m/s    
                node->children[ i ] = simulate( &copy_node, dt, Backwards);
                break;
            case Left_Backwards:                   // expand left backwards
                copy_node.state.phi = 0.10 ;       // 15º
                copy_node.state.speed = -1 ;       // move backwards at 1 m/s    
                node->children[ i ] = simulate( &copy_node, dt, Left_Backwards);
                break;
            case Right_Backwards:                  // expand right forwards
                copy_node.state.phi = -0.10 ;      // -15º
                copy_node.state.speed = -1 ;       // move backwards at 1 m/s    
                node->children[ i ] = simulate( &copy_node, dt, Right_Backwards);
                break;
        }
    
        //printf("%p \n", node->children[i]) ;
    }

}

node_t * find_best_node_rec( node_t * start, node_t * best ){

    int i = 0;

    for( i = 0 ; i < CHILDREN_NUM ; i++ ){
        
        if( start->children[i] != NULL ){   // if it has a valid children
            // look into it
            best = find_best_node_rec( start->children[i], best ) ; 
        }
        else{
            if( start->t_cost < best->t_cost ){     // found a better node
                best = start;       // update best node
            }      
        }
    }

    return best;
}

node_t * a_star( node_t * node, double dt ){

    int i;
    for(i = 0 ; i < CHILDREN_NUM ; i++)
        printf("[%p] \n", node->children[i] );

    // populate first children
    expand_node( node, dt ) ;
    
    for(i = 0 ; i < CHILDREN_NUM ; i++)
        printf("[%p] \n", node->children[i] );

    node_t * current_best_node = NULL;
    current_best_node = find_best_node_rec( node, node ) ;
    
    best_node = current_best_node;

    for( i = 0 ; i < 10000 ; i++ ){

        //printf("[%d] print possible nodes: \n", i);
        //print_possible_nodes_rec( node ) ;

        //for(i = 0 ; i < 4 ; i++)   
        //      print_node( node->children[i][0] );
            

        // look for the best costs recursively
        current_best_node = find_best_node_rec( node, node ) ;
        
        //printf("current_best : ");
        //print_node( best_node[0] );
        //print_node( current_best_node[0] );
        
        if( current_best_node->cost < best_node->cost ){
            best_node = current_best_node;
            print_node( best_node[0] );
        }

        expand_node( current_best_node, dt );
        

    }

    printf("\nbest all time node: \n");
    print_node( best_node[0] );

    printf("\nbest node now: \n");
    print_node( current_best_node[0] );
    
    return best_node ;
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  
  //rclcpp::spin(std::make_shared<MapSubscriber>());
 rclcpp::Rate loop_rate(100ms);
 auto node_sub = std::make_shared<MapSubscriber>() ;
  
   
  while(1){
    rclcpp::spin_some(node_sub);
    printf("heigth : %d, width : %d, resolution : %f, origin [x, y, theta] : [%f, %f, %f] \n",
		heigth, width, resolution, origin.x, origin.y, origin.theta);
    
    
    
    if( flag_new_map == 1 && flag_new_pose == 1 ){		// has new pose and map to work with
        // calculate next best pose
        printf("calculating new pose!!!! \n");
		
		// print map column
		int i;
		printf("MAP: \n");
		
		// print row
		for(i = 0 ; i < width ; i++){
			printf("%3d ", map_now[i] );
		}
		printf("\n");
		
		// current robot pose
		printf("robot_pose [x, y, theta] : [%f, %f, %f ] \n",
			robot_slam.x, robot_slam.y, robot_slam.theta );
		
        
       
        node_t start = { { { robot_slam.x, robot_slam.y, robot_slam.theta }, 0, 0, 0.2 }, 0, 50, 0, 1e3, {0}, NULL };
        best_node = &start ;
        //node_t start = create_node( { 2, 2, M_PI/2 }, 0, 0 );

        double dt = 1.0 ;

        //expand_node( &start, dt ) ;

        printf("%p \n", start.children[0]) ;
        cout << "start: \n";
        print_node( start );
        
        cout << "children: \n";
        
        
        node_t * final = a_star( &start, dt );

        printf("Hello World: \n");
        print_node(final[0]);
        
        flag_new_map = 0;
        flag_new_pose = 0;
        
        
        
    }
    else{
        loop_rate.sleep();
    }
  }
  
  
  
  return 0;
}

