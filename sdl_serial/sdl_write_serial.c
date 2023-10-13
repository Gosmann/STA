// this program should read a serial device of the Raspberry Pi
// (/dev/ttyUSB0 or /dev/ttyS0) and print in the terminal the communication

#include <stdio.h>
#include <string.h>
#include <fcntl.h> 	// Contains file controls like O_RDWR
#include <errno.h> 	// Error integer and strerror() function
#include <termios.h> 	// Contains POSIX terminal control definitions
#include <unistd.h> 	// write(), read(), close()
#include <time.h>

#include <SDL2/SDL.h>

#define SCREEN_HEIGHT   400
#define SCREEN_WIDTH    400
#define SCREEN_ORIGIN_X     100
#define SCREEN_ORIGIN_Y     100

#define MAX_BUFFER 256

int main(){

	// int serial_port = open("/dev/ttyUSB0", O_RDWR); 	// usb port
	int serial_port = open("/dev/ttyS0", O_RDWR);	// GPIO pins
	
	char buffer[MAX_BUFFER] ;	// buffer that stores the commands

	// serial configuration
	struct termios tty ;		// struct that store serial configuration
	tcgetattr(serial_port, &tty) ;	
	tty.c_cc[VTIME] = 0;		// no timeout delay	
	tty.c_cc[VMIN] = 0;		// no timeout delay
	cfsetispeed(&tty, B9600);	// define input speed
	cfsetospeed(&tty, B9600);	// define output speed
	
	if(serial_port < 0){
		printf("error oppening serial port \n" );
		return -1;
	}

	// SDL stuff
	SDL_Init( SDL_INIT_EVERYTHING ) ;
	const Uint8 *state = SDL_GetKeyboardState(NULL); 
	SDL_Event event;

	SDL_Window *window ;

	window = SDL_CreateWindow("test oppening screen", SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED,
		SCREEN_WIDTH, SCREEN_HEIGHT, SDL_WINDOW_RESIZABLE)  ; 

	if( window == NULL ){       // error opening window
		//cout << "error oppening screen! \n " << SDL_GetError();
	}

	char running = 1;

	while( running ){

		while( SDL_PollEvent( &event ) ){
		
			if( event.type ==  SDL_QUIT ){
                running = 0;
                //cout << "this is the end! \n" ;
                break;
            }

		int j = 0;

		//scanf("%s", buffer);
		SDL_PumpEvents();
		
		if (state[ SDL_SCANCODE_W ]) { 
			fflush(stdout);
			//cout << "W " ;
			printf("W ");

			//strcpy(buffer, "w \n\0");
			buffer[0] = 'w';
			buffer[1] = ' ';
			buffer[2] = '\n';
			buffer[3] = '\0';


			int n = write(serial_port, buffer, strlen(buffer));
			
			if(n < 0){
				printf("write with error \n");
			}
			else if(n == strlen(buffer)){
				printf("write successful \n");
			}
			else{
				printf("write strange \n");
			}
		
		}
	
		
		//printf("> [%s] \n", buffer);
		//sleep(1);
		}

	}
	
	SDL_Quit();

	return 0;
}
