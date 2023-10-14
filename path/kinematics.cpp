#include "kinematics.h"

void simulate( robot_t * robot, double dt ){

    robot->pose.x += robot->speed * cos( robot->pose.theta ) * dt ; 
    robot->pose.y += robot->speed * sin( robot->pose.theta ) * dt ; 
    robot->pose.theta += (robot->speed / robot->L) * tan( robot->phi ) * dt ; 

    //new_robot.pose.x = robot.pose.x + robot.speed * cos( robot.pose.theta ) * dt ;
    //new_robot.pose.y = robot.pose.y + robot.speed * sin( robot.pose.theta ) * dt ;
    //new_robot.pose.theta = robot.pose.theta + ( robot.speed / robot.L ) * tan ( robot.phi ) * dt ;

}

void show_position( SDL_Renderer * renderer, pose_t pose ){
    SDL_SetRenderDrawColor( renderer, 0, 0, 0, 255 );   // shows in black

    double max_y = 10 ;    // 10m max value for y
    double max_x = 10 ;    // 10m max value for x

    SDL_RenderDrawPoint( renderer, pose.x * (SCREEN_HEIGHT / max_x) , SCREEN_HEIGHT - pose.y * (SCREEN_HEIGHT / max_y) );
    SDL_RenderPresent( renderer );    

}

void show_path( SDL_Renderer * renderer, vector<pose_t> path  ){
    SDL_SetRenderDrawColor( renderer, 0, 0, 0, 255 );   // shows in black
    int i;

    double max_y = 10 ;    // 10m max value for y
    double max_x = 10 ;    // 10m max value for x

    
    for(i = 0 ; i < path.size() - 1 ; i++){
        SDL_RenderDrawLine( renderer, path[i].x * (SCREEN_HEIGHT / max_x) , SCREEN_HEIGHT - path[i].y * (SCREEN_HEIGHT / max_y) , 
            path[i+1].x * (SCREEN_HEIGHT / max_x) , SCREEN_HEIGHT - path[i+1].y * (SCREEN_HEIGHT / max_y) );
    } 
    
    //SDL_RenderDrawLine( renderer, 0, 0, 100, 100);
    SDL_RenderPresent( renderer );    
}

void print_pose( pose_t pose ){
    char buffer[100];
    sprintf(buffer, "[x, y, theta] : [%f, %f, %f] \n", pose.x, pose.y, pose.theta) ;
    cout << buffer ;
}

void print_pose_and_time( pose_t pose, double time){

    char buffer[100];
    sprintf(buffer, "[%1.2f] -> [x, y, theta] : [%f, %f, %f] \n", time, pose.x, pose.y, pose.theta) ;
    cout << buffer ;
}

