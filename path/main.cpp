#include <SDL2/SDL.h>
#include <iostream>
#include <vector>
using namespace std;

#include "display.h"
#include "kinematics.h"


void expand_node( node_t * node, double dt ){
    
    uint8_t actions[6] = { 'L', 'R', 'F', 'B', 'Z', 'X' } ;

    int i;
    for(i = 0 ; i < CHILDREN_NUM ; i++){
        
        node_t copy_node = node[0];
        
        switch( actions[i] ){
            case Left_Forwards:                    // expand left forwards
                copy_node.state.phi = 0.25 ;       // 15º
                copy_node.state.speed = 1 ;        // move forwards at 1 m/s    
                node->children[ i ] = simulate( &copy_node, dt, Left_Forwards);
                break;
            case Right_Forwards:                   // expand right forwards
                copy_node.state.phi = -0.25 ;      // -15º
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
                copy_node.state.phi = 0.25 ;       // 15º
                copy_node.state.speed = -1 ;       // move backwards at 1 m/s    
                node->children[ i ] = simulate( &copy_node, dt, Left_Backwards);
                break;
            case Right_Backwards:                  // expand right forwards
                copy_node.state.phi = -0.25 ;      // -15º
                copy_node.state.speed = -1 ;       // move backwards at 1 m/s    
                node->children[ i ] = simulate( &copy_node, dt, Right_Backwards);
                break;
        }
    
        //printf("%p \n", node->children[i]) ;
    }

}

node_t * best_node ;

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

        node->children[ 0 ] = simulate( &node_left, dt, Left_Forwards);
        expand_recursive( node->children[0], dt, max_depth );

        node->children[ 1 ] = simulate( &node_right, dt, Right_Forwards);
        expand_recursive( node->children[1], dt, max_depth );

        node->children[ 2 ] = simulate( &node_forwards, dt, Forwards);
        expand_recursive( node->children[2], dt, max_depth );

        node->children[ 3 ] = simulate( &node_backwards, dt, Backwards);
        expand_recursive( node->children[3], dt, max_depth );        

    }
    
    if( node->cost < best_node->cost ){
        print_node( node[0] );
        best_node = node ;
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

void print_possible_nodes_rec( node_t * start ){
    int i;

    print_node( start[0] ) ;

    for( i = 0 ; i < CHILDREN_NUM ; i++ ){
        
        if( start->children[i] != NULL ){   // if it has a valid children
            // look into it
            print_possible_nodes_rec( start->children[i] ) ; 
        }
    }

    
}

void a_star( node_t * node, double dt ){

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

    for( i = 0 ; i < 1000 ; i++ ){

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
    
}

int main( int argc, char ** argv ){

    SDL_Init( SDL_INIT_EVERYTHING ) ;

    SDL_Event event;
    SDL_Window * window = NULL;
    SDL_Renderer * renderer = NULL; 

    create_window( &window,  &renderer);

    //robot_t robot = { {2, 2, 1}, 0, 0, 1} ;
        
    node_t start = { { { 2, 2, M_PI/2 }, 0, 0, 1 }, 0, 1e3, 0, 1e3, {0}, NULL };
    best_node = &start ;
    //node_t start = create_node( { 2, 2, M_PI/2 }, 0, 0 );

    double dt = 1.0 ;

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
    
    */

    a_star( &start, dt );

    //expand_recursive( &start, dt, 12 );

    //printf("best node: \n");
    //print_node( best_node[0] );

    // simulate

    /*
    
    robot_t robot = { {2, 2, 3.14/2}, 0, 0, 1} ;
    node_t * dummy = &start ;
    //double dt2 = 10e-3;
    //double total_time = 1;

    vector<pose_t> path_mov ;

    for( i = 0 ; i < best_node[0].depth ; i++ ){
        
        switch(best_node[0].path[i]){
            case 'L':
                dummy->state.phi += 0.1 ;         // move 5 degrees to left
                break;
            case 'R':
                dummy->state.phi -= 0.1 ;        // move 5 degrees to left
                break;
            case 'F':
                dummy->state.speed = 1.0;     // move forwards at 1 m/s  
                break;
            case 'B':
                dummy->state.speed = -1.0;   // move forwards at 1 m/s
                break;
        }

        dummy = simulate( dummy, dt, Left ) ;     

        print_pose_and_time( dummy->state.pose, (i * dt));

        path_mov.push_back( dummy->state.pose );
        //show_position(renderer, robot.pose);
    
        
    }
    show_path( renderer, path_mov );

    printf("[%d] ", sizeof(node_t));
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
        
        running = false;

        

    }




    SDL_DestroyWindow(window);
    SDL_Quit();    

    return 0;
}

