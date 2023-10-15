#include "kinematics.h"

void simulate_robot( robot_t * robot, double dt ){

    robot->pose.x += robot->speed * cos( robot->pose.theta ) * dt ; 
    robot->pose.y += robot->speed * sin( robot->pose.theta ) * dt ; 
    robot->pose.theta += (robot->speed / robot->L) * tan( robot->phi ) * dt ; 
    
}


node_t * simulate( node_t * node, double dt, actions act ){

    node_t * p_node = (node_t *)malloc( sizeof(node_t) );
    p_node->state.speed = node->state.speed;
    p_node->state.phi = node->state.phi;
    p_node->state.L = node->state.L;
    p_node->cost = node->cost;
    
    for(int i = 0 ; i < node->depth ; i++)
        p_node->path[i] = node->path[i];

    p_node->path[node->depth] = act ;

    //robot->pose.x += robot->speed * cos( robot->pose.theta ) * dt ; 
    p_node->state.pose.x = node->state.pose.x + node->state.speed * cos( node->state.pose.theta ) * dt ;
    
    //robot->pose.y += robot->speed * sin( robot->pose.theta ) * dt ; 
    p_node->state.pose.y = node->state.pose.y + node->state.speed * sin( node->state.pose.theta ) * dt ;

    //robot->pose.theta += (robot->speed / robot->L) * tan( robot->phi ) * dt ; 
    p_node->state.pose.theta = node->state.pose.theta + ( node->state.speed / node->state.L ) * tan( node->state.phi ) * dt ;

    p_node->depth = node->depth + 1;    // increase depth

    return p_node;
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

void print_state( robot_t state ){
    char buffer[100];
    sprintf(buffer, "[x, y, theta, speed, phi] : [%f, %f, %f, %f, %f] \n",
        state.pose.x, state.pose.y, state.pose.theta, state.speed, state.phi ) ;
    cout << buffer ;
}

void print_node( node_t node ){
    /*
    int j;
    for(j = 0 ; j < node.depth ; j++)
        printf("\t");
    for(j = 0 ; j < node.depth ; j++)
        printf("[%c]", node.path[j]);
    
    
    char buffer[200];
    
    sprintf(buffer, "[x, y, theta, speed, phi, depth, cost] : [%8.5f, %8.5f, %1.2f, %5.2f, %5.2f, %2d, %4.2f] \n",
        node.state.pose.x, node.state.pose.y, node.state.pose.theta, 
        node.state.speed, node.state.phi, node.depth, node.cost) ;
    cout << buffer ;
    // */
}

void print_all_children( node_t * node ){
    int i, j;
    for(i = 0 ; i < 4 ; i++){
        
        if( node->children[i] != NULL ){
            print_node( node->children[i][0] );
        }
            
    }
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

