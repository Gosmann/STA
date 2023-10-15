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

    expand_node( &start, dt ) ;

    printf("%p \n", start.children[0]) ;
    cout << "start: \n";
    print_node( start );
    
    cout << "children: \n";
    expand_node( &start, dt ) ;
    print_all_children( &start );

    int i, j, k, l, m, n, o, p, q, r, s;
    bool running = false ;
    

    cout << "new layer \n" ;
    for( i = 0 ; i < 4 ; i++ ){
        expand_node( start.children[i], dt ) ;
        //print_all_children( &start.children[i][0] );
    }

    cout << "new new layer \n" ;
    for( i = 0 ; i < 4 ; i++ ){
        for(j = 0 ; j < 4 ; j++){
            expand_node( start.children[i]->children[j], dt ) ;
            //print_all_children( &start.children[i]->children[j][0] );
        }
    }

    cout << "new new new layer \n" ;
    for( i = 0 ; i < 4 ; i++ ){
        for(j = 0 ; j < 4 ; j++){
            for(k = 0 ; k < 4 ; k++){
                expand_node( start.children[i]->children[j]->children[k], dt ) ;
                //print_all_children( &start.children[i]->children[j]->children[k][0] );
            }
        }
    }

    cout << "new new new new layer \n" ;
    for( i = 0 ; i < 4 ; i++ ){
        for(j = 0 ; j < 4 ; j++){
            for(k = 0 ; k < 4 ; k++){
                for(l = 0 ; l < 4 ; l++){
                    expand_node( start.children[i]->children[j]->children[k]->children[l], dt ) ;
                    //print_all_children( &start.children[i]->children[j]->children[k]->children[l][0] );
                }
            }
        }
    }

    cout << "new new new new new layer \n" ;
    for( i = 0 ; i < 4 ; i++ ){
        for(j = 0 ; j < 4 ; j++){
            for(k = 0 ; k < 4 ; k++){
                for(l = 0 ; l < 4 ; l++){
                    for(m = 0 ; m < 4 ; m++){
                        expand_node( start.children[i]->children[j]->children[k]->children[l]->children[m], dt ) ;
                        //print_all_children( &start.children[i]->children[j]->children[k]->children[l]->children[m][0] );
                    }
                }
            }
        }
    }

    cout << "new new new new new new layer \n" ;
    for( i = 0 ; i < 4 ; i++ ){
        for(j = 0 ; j < 4 ; j++){
            for(k = 0 ; k < 4 ; k++){
                for(l = 0 ; l < 4 ; l++){
                    for(m = 0 ; m < 4 ; m++){
                        for(n = 0 ; n < 4 ; n++){
                            expand_node( start.children[i]->children[j]->children[k]->children[l]->children[m]->children[n], dt ) ;
                            //print_all_children( &start.children[i]->children[j]->children[k]->children[l]->children[m]->children[n][0] );
                        }
                    }
                }
            }
        }
    }
        
    cout << "new new new new new new new layer \n" ;
    for( i = 0 ; i < 4 ; i++ ){
        for(j = 0 ; j < 4 ; j++){
            for(k = 0 ; k < 4 ; k++){
                for(l = 0 ; l < 4 ; l++){
                    for(m = 0 ; m < 4 ; m++){
                        for(n = 0 ; n < 4 ; n++){
                            for(o = 0 ; o < 4 ; o++){
                                expand_node( start.children[i]->children[j]->children[k]->children[l]->children[m]->children[n]->children[o], dt ) ;
                                //print_all_children( &start.children[i]->children[j]->children[k]->children[l]->children[m]->children[n]->children[o][0] );
                            }
                        }
                    }
                }
            }
        }
    }

    cout << "new new new new new new new new layer \n" ;
    for( i = 0 ; i < 4 ; i++ ){
        for(j = 0 ; j < 4 ; j++){
            for(k = 0 ; k < 4 ; k++){
                for(l = 0 ; l < 4 ; l++){
                    for(m = 0 ; m < 4 ; m++){
                        for(n = 0 ; n < 4 ; n++){
                            for(o = 0 ; o < 4 ; o++){
                                for(p = 0 ; p < 4 ; p++){
                                    expand_node( start.children[i]->children[j]->children[k]->children[l]->children[m]->children[n]->children[o]->children[p], dt ) ;
                                    //print_all_children( &start.children[i]->children[j]->children[k]->children[l]->children[m]->children[n]->children[o]->children[p][0] );
                                }
                            }
                        }
                    }
                }
            }
        }
    }

    cout << "new new new new new new new new new layer \n" ;
    for( i = 0 ; i < 4 ; i++ ){
        for(j = 0 ; j < 4 ; j++){
            for(k = 0 ; k < 4 ; k++){
                for(l = 0 ; l < 4 ; l++){
                    for(m = 0 ; m < 4 ; m++){
                        for(n = 0 ; n < 4 ; n++){
                            for(o = 0 ; o < 4 ; o++){
                                for(p = 0 ; p < 4 ; p++){
                                    for(q = 0 ; q < 4 ; q++){
                                        expand_node( start.children[i]->children[j]->children[k]->children[l]->children[m]->children[n]->children[o]->children[p]->children[q], dt ) ;
                                        print_all_children( &start.children[i]->children[j]->children[k]->children[l]->children[m]->children[n]->children[o]->children[p]->children[q][0] );
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
    }

    cout << "new new new new new new new new new new layer \n" ;
    for( i = 0 ; i < 4 ; i++ ){
        for(j = 0 ; j < 4 ; j++){
            for(k = 0 ; k < 4 ; k++){
                for(l = 0 ; l < 4 ; l++){
                    for(m = 0 ; m < 4 ; m++){
                        for(n = 0 ; n < 4 ; n++){
                            for(o = 0 ; o < 4 ; o++){
                                for(p = 0 ; p < 4 ; p++){
                                    for(q = 0 ; q < 4 ; q++){
                                        for(r = 0 ; r < 4 ; r++){
                                            expand_node( start.children[i]->children[j]->children[k]->children[l]->children[m]->children[n]->children[o]->children[p]->children[q]->children[r], dt ) ;
                                            print_all_children( &start.children[i]->children[j]->children[k]->children[l]->children[m]->children[n]->children[o]->children[p]->children[q]->children[r][0] );
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
    }

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

