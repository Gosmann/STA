#include <SDL2/SDL.h>

#include <iostream>
using namespace std;

#define SCREEN_HEIGHT   400
#define SCREEN_WIDTH    400
#define SCREEN_ORIGIN_X     100
#define SCREEN_ORIGIN_Y     100

int main( int argc, char *argv[] ){

    cout << "Hello World! \n" ;  
    char input;

    SDL_Init( SDL_INIT_EVERYTHING ) ;

    SDL_Window *window ;

    window = SDL_CreateWindow("test oppening screen", SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED,
        SCREEN_WIDTH, SCREEN_HEIGHT, SDL_WINDOW_RESIZABLE)  ; 

    if( window == NULL ){       // error opening window
        cout << "error oppening screen! \n " << SDL_GetError();
    }

    while(1){
        cin >> input; 
        
        if( input == 'F'){
            break;
        }

    }

    SDL_DestroyWindow(window);

    SDL_Quit();

    return 0;
}