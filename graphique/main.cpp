#include <SDL2/SDL.h>
#include <iostream>
using namespace std;

#include "display.h"

int main( int argc, char *argv[] ){

    SDL_Init( SDL_INIT_EVERYTHING ) ;

    SDL_Event event;
    SDL_Window * window = NULL;
    SDL_Renderer * renderer = NULL; 
    
    create_window( &window, &renderer );

    bool running = true;
    uint32_t counter;

    SDL_Color color = {0, 0, 0} ;
    draw_row( renderer, 10, color);
    
    SDL_RenderClear( renderer );
    SDL_SetRenderDrawColor( renderer, 255, 255, 255, 255 );
    //SDL_SetRenderDrawColor( renderer, 255, 255, 255, 255 );

    for(int i = 0 ; i < SCREEN_WIDTH ; i++){
        SDL_RenderDrawPoint( renderer, i, 10 );
    }
    
    SDL_RenderPresent( renderer ) ;


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

    
    close_window( window );
    SDL_Quit();

    return 0;
}