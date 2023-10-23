#include <SDL2/SDL.h>
#include "display.h"

void create_window( SDL_Window ** window_,  SDL_Renderer ** renderer_){
    
    SDL_Window * window = window_[0];
    SDL_Renderer * renderer = renderer_[0]; 
    
    SDL_CreateWindowAndRenderer(SCREEN_WIDTH, SCREEN_HEIGHT, 0, &window, &renderer) ; 
    
    SDL_SetRenderDrawColor( renderer, 0, 255, 0, 255 ); 
    SDL_RenderClear( renderer );

    //SDL_SetRenderDrawColor( renderer, 255, 255, 255, 255 ); 
    //SDL_RenderDrawPoint( renderer, 10, 10 );
    SDL_RenderPresent( renderer );

}

void draw_row(SDL_Renderer * renderer, uint32_t row, SDL_Color color){
    
    SDL_RenderClear( renderer );
    SDL_SetRenderDrawColor( renderer, color.r, color.g, color.b, 255 );
    //SDL_SetRenderDrawColor( renderer, 255, 255, 255, 255 );

    for(int i = 0 ; i < SCREEN_WIDTH ; i++){
        SDL_RenderDrawPoint( renderer, i, row );
    }
    
    SDL_RenderPresent( renderer ) ;
}

void close_window( SDL_Window * window){
    SDL_DestroyWindow(window);
}