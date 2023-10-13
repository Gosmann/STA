// this program should read a serial device of the Raspberry Pi
// (/dev/ttyUSB0 or /dev/ttyS0) and print in the terminal the communication

#include <stdio.h>
#include <string.h>
#include <fcntl.h> 	// Contains file controls like O_RDWR
#include <errno.h> 	// Error integer and strerror() function
#include <termios.h> 	// Contains POSIX terminal control definitions
#include <unistd.h> 	// write(), read(), close()
#include <time.h>

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
	
	while(1){
		int j = 0;

		scanf("%s", buffer);
		
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
		
		//printf("> [%s] \n", buffer);
		//sleep(1);
	}
	
	return 0;
}
