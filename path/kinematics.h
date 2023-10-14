#include <math.h>      
#include <SDL2/SDL.h> 
#include <iostream>
#include <vector>
using namespace std;

#include "display.h"

typedef struct {
    double x;
    double y;
    double theta;
} pose_t, *p_pose_t;

typedef struct {
    pose_t pose;
    double phi ;
    double speed ;
    const double L ;
} robot_t, *p_robot_t;

void simulate( robot_t * robot, double dt );
void show_position( SDL_Renderer * renderer, pose_t pose );
void show_path( SDL_Renderer * renderer, vector<pose_t> path  );

void print_pose( pose_t pose );
void print_pose_and_time( pose_t pose, double time);

