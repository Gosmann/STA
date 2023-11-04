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
	int serial_port = open("/dev/ttyS0", O_RDWR);	// GPIO pins (Arduino mega Serial2)
	//int serial_port = open("/dev/ttyACM0", O_RDWR);	// GPIO pins (LIDAR)
	
	char buffer[MAX_BUFFER] ;	// buffer that stores the commands

	// serial configuration
	struct termios tty ;		// struct that store serial configuration
	tcgetattr(serial_port, &tty) ;	
	tty.c_cc[VTIME] = 0;		// no timeout delay	
	tty.c_cc[VMIN] = 0;		// no timeout delay
	
	cfsetispeed(&tty, B115200);	// define input speed
	cfsetospeed(&tty, B115200);	// define output speed
	
	if(serial_port < 0){
		printf("error oppening serial port \n" );
		return -1;
	}
	

	sprintf(buffer, "%c%c\n\0", 0xA5, 0x60);
	int n = write(serial_port, buffer, 4);

	printf("%d \n", n);

	while(1){
		int j = 0;

		while( j < MAX_BUFFER-1 ){

			int n = read(serial_port, &buffer[j], 1);
			if( n != 1){
				// notify error
				//printf("ERROR reading serial \n");
			}
			else if( buffer[j] == '\n' && n == 1 ){
				// end of the command
				buffer[j + 0] = '\0' ; // make line termination
				break;
			}
			else{
				// continues reading 1 char at a time
				j++;
			}
			
		}
		
		printf("> [%s] \n", buffer);
		//sleep(1);
	}
	
	return 0;
}
