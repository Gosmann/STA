#include <SDL2/SDL.h>
#include <iostream>
#include <vector>
using namespace std;

#include "display.h"
#include "kinematics.h"



int main( int argc, char ** argv ){

    SDL_Init( SDL_INIT_EVERYTHING ) ;

    SDL_Event event;
    SDL_Window * window = NULL;
    SDL_Renderer * renderer = NULL; 

    create_window( &window,  &renderer);

    robot_t robot = { {2, 2, 1}, 0, 0, 1} ;
    pose_t target = {1, 0, 0} ;

    cout << "start : \n";
    print_pose( robot.pose );

    cout << "target : \n";
    print_pose( target );

    int i;
    bool running = true ;
    double dt = 10e-3;
    double total_time = 1;

    robot.speed = 15;
    robot.phi = -0.25;

    vector<pose_t> path ;

    for( i = 0 ; i < uint32_t(total_time/dt) ; i++ ){

        print_pose_and_time( robot.pose, (i * dt));
        path.push_back( robot.pose );
        //show_position(renderer, robot.pose);
        
        simulate( &robot, dt ) ;            
    }
    show_path( renderer, path );

    path.clear();

    robot_t robot2 = { {2, 2, 0}, 0.25, 10, 1} ;
    for( i = 0 ; i < uint32_t(total_time/dt) ; i++ ){

        print_pose_and_time( robot2.pose, (i * dt));
        path.push_back( robot2.pose );
        //show_position(renderer, robot.pose);
        
        simulate( &robot2, dt ) ;            
    }
    show_path( renderer, path );

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
        

        

    }




    SDL_DestroyWindow(window);
    SDL_Quit();    

    return 0;
}

