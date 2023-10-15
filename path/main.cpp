#include <SDL2/SDL.h>
#include <iostream>
#include <vector>
using namespace std;

#include "display.h"
#include "kinematics.h"


void expand_node( node_t * node, double dt ){
    uint8_t actions[4] = { 'L', 'R', 'F', 'B' } ;

    int i;
    for(i = 0 ; i < 4 ; i++){
        node_t copy_node = node[0];
        
        switch( actions[i] ){
            case Left:          // expand left
                copy_node.state.phi += 0.1 ;    // move 5 degrees to left
                node->children[ i ] = simulate( &copy_node, dt, Left);
                break;
            case Right:         // expand right
                copy_node.state.phi -= 0.1 ;    // move 5 degrees to right
                node->children[ i ] = simulate( &copy_node, dt, Right);
                break;
            case Forwards:      // expand up
                copy_node.state.speed = 1 ;     // move forwards at 1 m/s
                node->children[ i ] = simulate( &copy_node, dt, Forwards);
                break;
            case Backwards:     // expand down
                copy_node.state.speed = -1 ;    // move backwards at 1 m/s
                node->children[ i ] = simulate( &copy_node, dt, Backwards);
                break;
        }
    
        //printf("%p \n", node->children[i]) ;
    }

}

double best = 10e3 ;

//expand_recursive( &start, dt, 5 );
void expand_recursive( node_t * node, double dt, uint32_t max_depth ){

    // stops recursion
    if( node->depth < max_depth ){      
        
        //print_node( node[0] );
        
        // creates new nodes
        node_t node_left = node[0] ; 
        node_t node_right = node[0] ; 
        node_t node_forwards = node[0] ; 
        node_t node_backwards = node[0] ; 

        node_left.state.phi += 0.1 ;         // move 5 degrees to left
        node_right.state.phi -= 0.1 ;        // move 5 degrees to left
        node_forwards.state.speed = 1.0;     // move forwards at 1 m/s
        node_backwards.state.speed = -1.0;   // move forwards at 1 m/s

        node->children[ 0 ] = simulate( &node_left, dt, Left);
        expand_recursive( node->children[0], dt, max_depth );

        node->children[ 1 ] = simulate( &node_right, dt, Right);
        expand_recursive( node->children[1], dt, max_depth );

        node->children[ 2 ] = simulate( &node_forwards, dt, Forwards);
        expand_recursive( node->children[2], dt, max_depth );

        node->children[ 3 ] = simulate( &node_backwards, dt, Backwards);
        expand_recursive( node->children[3], dt, max_depth );        

    }
    else{
        if( node->cost < best ){
            print_node( node[0] );
            best = node->cost ;
        }
        
    }
}

int main( int argc, char ** argv ){

    SDL_Init( SDL_INIT_EVERYTHING ) ;

    SDL_Event event;
    SDL_Window * window = NULL;
    SDL_Renderer * renderer = NULL; 

    create_window( &window,  &renderer);

    //robot_t robot = { {2, 2, 1}, 0, 0, 1} ;
        
    node_t start = { { { 2, 2, M_PI/2 }, 0, 0, 1 }, 0, 0, {0}, NULL };

    //node_t start = create_node( { 2, 2, M_PI/2 }, 0, 0 );

    double dt = 0.5 ;

    //expand_node( &start, dt ) ;

    printf("%p \n", start.children[0]) ;
    cout << "start: \n";
    print_node( start );
    
    cout << "children: \n";
    //expand_node( &start, dt ) ;
    //print_all_children( &start );
    //expand_recursive( &start, dt, 10 );

    int i, j;
    bool running = true ;
    
    /*
    cout << "new layer \n" ;
    for( i = 0 ; i < 4 ; i++ ){
        expand_node( start.children[i], dt ) ;
        //print_all_children( &start.children[i][0] );
    }
    */



    // simultion stuff

    /*
    robot_t robot = { {6, 9, -3.14/2}, 3.14/9, 10, 1} ;
    double dt = 10e-3;
    double total_time = 1;

    vector<pose_t> path ;

    for( i = 0 ; i < uint32_t(total_time/dt) ; i++ ){
        if( i % 5 == 0)
            print_pose_and_time( robot.pose, (i * dt));

        path.push_back( robot.pose );
        //show_position(renderer, robot.pose);
        
        if( i ==  uint32_t((total_time/dt)*0.5) ){
            robot.phi *= -1 ;
            robot.speed *= 3 ;
        }

        simulate_robot( &robot, dt ) ;            
    }
    show_path( renderer, path );
    */

    printf("[%d] ", sizeof(node_t));

    while( running ){
        
        // deals with closing the screen
        while( SDL_PollEvent( &event ) ){     
        
            if( event.type ==  SDL_QUIT ){
                running = false;
                cout << "this is the end! \n" ;
                break;
            }

        }

        // main program loop 
        expand_recursive( &start, dt, 12 );

        running = false;

        

    }




    SDL_DestroyWindow(window);
    SDL_Quit();    

    return 0;
}

