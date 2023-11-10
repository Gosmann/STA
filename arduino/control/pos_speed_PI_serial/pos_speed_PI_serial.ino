// ATTENTION
// to get scanf and printf to work with floats on Arduino, edit the file "platform.txt" in directory
// pi@raspberrypi:/usr/share/arduino/hardware/arduino/avr
// goto lines with "compiler.c.extra_flags=" and "compiler.c.elf.extra_flags=" and add the following
// =-Wl,-u,vfprintf -lprintf_flt -lm -Wl,-u,vfscanf -lscanf_flt -lm
// then the printing will work fine


#include "encoders.hpp"
#define ENCODER_OPTIMIZE_INTERRUPTS

#include <Encoder.h>
//#include "Encoder.h"

#include "interrupts.hpp"
#include "motors.hpp"
#include "control.hpp"
#include <stdio.h>

#define BUFFER_SIZE 10

char buffer[128] = {0};
char buffer2[128] = {0};

typedef struct robot_t{

	char msg[40];
	encoder_t encoder_power ;
	encoder_t encoder_direc ;
	
} robot_t ;


//static robot_t ;
robot_t mobile_robot ;

static motor_t motor_power = init_motor( 12, 34, 35 );
static motor_t motor_direc = init_motor( 8, 37, 36 );

static encoder_t encoder_power = init_encoder(  18, 3 ); // 18 31
static encoder_t encoder_direc = init_encoder(  19, 2 ); // 19 38

Encoder encoder_lib_power( (int8_t)encoder_power.A, (int8_t)encoder_power.B );
Encoder encoder_lib_direc( (int8_t)encoder_direc.A, (int8_t)encoder_direc.B );

int position = 0;

static control_t control_direc_omega = init_control( 15, 0.15) ;   	// direction internal speed control
static control_t control_direc_theta = init_control( 2, 0) ;      	// direction external position control 

static control_t control_power_omega = init_control( 5, 0.15) ;   // power internal speed control
static control_t control_power_theta = init_control( 1, 0) ;       // power external position control 

static float time_counter = 0;
static uint32_t cnt = 0;

char buffer_serial[100] = {0};

// put your setup code here, to run once:
void setup() {

  	Serial.begin(115200);        	// /dev/ttyUSB0   usb cable serial
  	Serial2.begin(115200);          // /dev/ttyS0     gpios on rpi
	
	pinMode(13, OUTPUT);            // blue LED

    //start_encoders_interrupt();
    start_timer_interrupt();        // starts 10ms timer interrupt
    
	// print header
	sprintf(buffer, "Hello World! \n");
    Serial2.print(buffer);  
		
}

/*
// put your main code here, to run repeatedly:
void loop() {
	int i;
	char buffer_serial[100] = {0};
	
	if( Serial.available() > 0 ){
		for(i = 0 ; i < 100 ; i++){
			char input = Serial.read() ;

			buffer_serial[i] = input;
			
			if( input == '\n' || input == '\0' || Serial.available() == 0 ){
				break;
			}
			
		}

		Serial.println(buffer_serial);

		if( buffer_serial[0] == 's' ){			// it is a command to set a parameter
			if( buffer_serial[2] == 'd' ){		// set the direction 
				
				float target ;
				sscanf( &buffer_serial[3], "%f", &target ) ;

				sprintf( buffer, "%1.5f \n", target) ;
				if( abs(target) <= 1.001){
				//if( 1 ){
					Serial.print( target ) ;
					control_direc_theta.set_point = target ;	
				}
				else{
					Serial.print( "target out of range (-1~1) [rad] \n" ) ;
				}
				
				
			}
			else if( buffer_serial[2] == 'p' ){		// set the direction	 
				
				float target ;
				sscanf( &buffer_serial[3], "%f", &target ) ;

				sprintf( buffer, "%1.5f \n", target) ;
				//if( abs(target) <= 1.001){
				if( 1 ){
					Serial.println( target ) ;
					control_power_theta.set_point += target ;	
				}
				else{
					Serial.print( "target out of range (-1~1) [rad] \n" ) ;
				}
				
				
			}
		}
	}

	// read the debounced value of the encoder button
  bool pb = encoder.button();

  // get the encoder variation since our last check, it can be positive or negative, or zero if the encoder didn't move
  // only call this once per loop cicle, or at any time you want to know any incremental change
  int delta = encoder.delta();

  // add the delta value to the variable you are controlling
  //myEncoderControlledVariable += delta;
  position += delta;
	
	sprintf(buffer, "$ [%5d] [%7.3d] ; \n", cnt, position ) ;
	//drive_voltage( motor_direc, 1.0 );
	
	//sprintf(buffer, "$ [%010d] [%s] ; \n\0", cnt, buffer_odom) ;
	
	cnt++;
	

	char msg_local[40] = {0} ;
	sprintf(msg_local, "$ [%10d] Hello ; \0", cnt);
	
	mobile_robot.encoder_power = encoder_power ;
	mobile_robot.encoder_direc = encoder_direc ;
	strcpy( mobile_robot.msg, msg_local);
	
	/*
	sprintf(buffer, "$ [%010d] [%s] ; \n\0", cnt, (( uint8_t * ) &mobile_robot) ) ;
	
	memcpy(& buffer[16] , &mobile_robot, sizeof(mobile_robot)) ;
	buffer[16 + sizeof(mobile_robot) + 1] = ']';
	buffer[16 + sizeof(mobile_robot) + 2] = ';';
	buffer[16 + sizeof(mobile_robot) + 3] = '\n';
	buffer[16 + sizeof(mobile_robot) + 4] = '\0';
	
	
	// sends bin data
	Serial2.write( (byte *) &mobile_robot, sizeof(robot_t) );	
	//Serial2.write( mobile_robot, sizeof(mobile_robot) );	
	Serial.println( buffer );
	//Serial.println( sizeof(robot_t));
	
  	digitalWrite(13, HIGH);
  	delay(80);
  	
	
}
*/
// loop code


void loop()
{
  	int i;
	
	
	if( Serial.available() > 0 ){
		for(i = 0 ; i < 100 ; i++){
			char input = Serial.read() ;

			buffer_serial[i] = input;
			
			if( input == '\n' || input == '\0' || Serial.available() == 0 ){
				break;
			}
			
		}

		Serial.println(buffer_serial);

		if( buffer_serial[0] == 's' ){			// it is a command to set a parameter
			if( buffer_serial[2] == 'd' ){		// set the direction 
				
				float target = 0 ;
				sscanf( &buffer_serial[0], "s d %f", &target ) ;

				sprintf( buffer, "%1.5f \n", target) ;
				if( abs(target) <= 1.001){
				//if( 1 ){
					Serial.print( target ) ;
					control_direc_theta.set_point = target ;	
				}
				else{
					Serial.print( "target out of range (-1~1) [rad] \n" ) ;
				}
				
				
			}
			else if( buffer_serial[2] == 'p' ){		// set the direction	 
				
				float target ;
				sscanf( &buffer_serial[0], "s p %f", &target ) ;

				sprintf( buffer, "%1.5f \n", target) ;
				if( abs(target) <= 35.001){
				//if( 1 ){
					Serial.println( target ) ;
					control_power_theta.set_point += (float)target ;	
				}
				else{
					Serial.print( "target out of range (-1~1) [rad] \n" ) ;
				}
				
				
			}
		}
	}
  

	// update odom_counters
  	//encoder_power.odom += (int32_t) encoder1.delta();
 	//encoder_direc.odom += (int32_t) encoder2.delta();
  	
  	// do stuff with the updated value
  	//sprintf(buffer2, "%ld %ld \n", encoder_power.odom, encoder_direc.odom ) ;
  	sprintf(buffer2, "%f %f \n", encoder_power.theta, encoder_direc.theta ) ;
  	Serial.print(buffer2);
  	//Serial.println(  );

	char msg_local[40] = {0} ;
	sprintf(msg_local, "$ [%10d] Hello ; \0", cnt);
	
	mobile_robot.encoder_power = encoder_power ;
	mobile_robot.encoder_direc = encoder_direc ;
	strcpy( mobile_robot.msg, msg_local);
	
	//sprintf(buffer, "$ [%010d] [%s] ; \n\0", cnt, (( uint8_t * ) &mobile_robot) ) ;
	
	memcpy(& buffer[16] , &mobile_robot, sizeof(mobile_robot)) ;
	buffer[16 + sizeof(mobile_robot) + 1] = ']';
	buffer[16 + sizeof(mobile_robot) + 2] = ';';
	buffer[16 + sizeof(mobile_robot) + 3] = '\n';
	buffer[16 + sizeof(mobile_robot) + 4] = '\0';
	
	
	// sends bin data
	Serial2.write( (byte *) &mobile_robot, sizeof(robot_t) );	

  	delay(100);
  	
}
 
//timer5 interrupt 10ms 
ISR(TIMER5_COMPA_vect){
	 
	// update odom_counters
  	encoder_power.odom = encoder_lib_power.read() ;
 	encoder_direc.odom = encoder_lib_direc.read() ;
	
	time_counter += 0.010 ; 		// time_counter is in seconds

	 {   // direc theta (position) control
    
        // calculates instant position [rad]
        calculate_pos(&encoder_direc);

        // calculates PID
        calculate_pid(&control_direc_theta, encoder_direc.theta ) ;

        // updates speed set point
        control_direc_omega.set_point = control_direc_theta.pid ;
    }  
	
    {   // direc omega (speed) control
    
    	// calculates instant speed [rad/s]
        calculate_speed( &encoder_direc );
    
        // calculates PID
        calculate_pid( &control_direc_omega, encoder_direc.omega_mean) ;

        // updates motor voltage
		drive_voltage( motor_direc, control_direc_omega.pid );
    }


    // control power
    {   // direc theta (position) control
    
        // calculates instant position [rad]
        calculate_pos(&encoder_power);

        // calculates PID
        calculate_pid(&control_power_theta, encoder_power.theta ) ;

        // updates speed set point
        control_power_omega.set_point = control_power_theta.pid ;
    }  
	
    {   // direc omega (speed) control
    
    	// calculates instant speed [rad/s]
        calculate_speed( &encoder_power );
    
        // calculates PID
        calculate_pid( &control_power_omega, encoder_power.omega_mean) ;

        // updates motor voltage
        drive_voltage( motor_power, control_power_omega.pid );
    }

        
}
