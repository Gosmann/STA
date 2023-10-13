#include <stdio.h>
#include <string.h>
#include <fcntl.h> // Contains file controls like O_RDWR
#include <errno.h> // Error integer and strerror() function
#include <termios.h> // Contains POSIX terminal control definitions
#include <unistd.h> // write(), read(), close()
#include <time.h>

int main(){

	int serial = open("/dev/ttyUSB0", O_RDWR);
	char read_buf[64] ;

	struct termios tty ;

	tcgetattr(serial, &tty) ;
	
	tty.c_cc[VTIME] = 0;    // Wait for up to 1s (10 deciseconds), returning as soon as any data is received.
	tty.c_cc[VMIN] = 0;

	cfsetispeed(&tty, B9600);
	cfsetospeed(&tty, B9600);	
	
	if(serial < 0){
		printf("error oppening serial port \n" );
		return -1;
	}
	
	while(1){
		int j = 0;

		while( j < sizeof(read_buf)-1 ){
			int n = read(serial, &read_buf[j], 1);
			if( n != 1){
				// notify error
				//printf("ERROR reading serial \n");
			}
			else if( read_buf[j] == '\n' && n == 1 ){
				// end of the command
				read_buf[j + 0] = '\0' ; // make line termination
				break;
			}
			else{
				// continues reading 1 char at a time
				j++;
			}
			
		}
		
		printf("Hello World [%s] \n", read_buf);
		//sleep(1);
	}
	
	return 0;
}
