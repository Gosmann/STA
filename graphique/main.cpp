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

    SDL_Event event;

    char running = 1;
    const Uint8 *state = SDL_GetKeyboardState(NULL); 

    while( running ){
        
        while( SDL_PollEvent( &event ) ){
        
            if( event.type ==  SDL_QUIT ){
                running = 0;
                cout << "this is the end! \n" ;
                break;
            }

            /*
            else if( event.type == SDL_KEYDOWN ){
                
                switch( event.key.keysym.sym ){
                    case SDLK_w: 
                        cout << "W is down! \n" ;
                        break;
                    case SDLK_a: 
                        cout << "A is down! \n" ;
                        break;
                    case SDLK_s: 
                        cout << "S is down! \n" ;
                        break;
                    case SDLK_d: 
                        cout << "D is down! \n" ;
                        break;
                    default:
                        cout << "other key is down! \n" ;            
                }
            }
            */

            SDL_PumpEvents();
            if (state[ SDL_SCANCODE_W ]) { 
                fflush(stdout);
                //cout << "W " ;
                printf("W ");
            }
                
        }

        // main program loop



    }

    SDL_DestroyWindow(window);

    SDL_Quit();

    return 0;
}