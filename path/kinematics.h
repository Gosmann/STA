#include <math.h>      
#include <SDL2/SDL.h> 
#include <iostream>
#include <vector>
using namespace std;

#include "display.h"

#define MAX_DEPTH 100
#define CHILDREN_NUM 6

enum actions {
    Left_Forwards = 'L', 
    Right_Forwards = 'R', 
    Forwards = 'F', 
    Backwards = 'B',
    Left_Backwards = 'Z', 
    Right_Backwards = 'X', 
} ;

typedef struct {
    double x;
    double y;
    double theta;
} pose_t, *p_pose_t;

typedef struct {
    pose_t pose;
    double phi ;
    double speed ;
    double L ;
} robot_t, *p_robot_t;

typedef struct node_t{

    robot_t state ;
    uint32_t depth ;
    double cost ;       // TODO change this to h_cost
    double g_cost ;
    double t_cost ; 
    
    uint8_t path[MAX_DEPTH] ; 
    
    struct node_t * children[CHILDREN_NUM] ;     // left, right, up, down
                                    // TODO create an enum for this
} node_t ;

// void simulate( robot_t * robot, double dt );
node_t * simulate( node_t * node, double dt, actions act);
void simulate_robot( robot_t * robot, double dt );

void show_position( SDL_Renderer * renderer, pose_t pose );
void show_path( SDL_Renderer * renderer, vector<pose_t> path  );

void print_pose( pose_t pose );
void print_state( robot_t state );
void print_node( node_t node );
void print_all_children( node_t * node );

void print_pose_and_time( pose_t pose, double time);

