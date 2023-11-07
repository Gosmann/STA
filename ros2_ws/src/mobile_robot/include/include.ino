// ATTENTION
// to get scanf and printf to work with floats on Arduino, edit the file "platform.txt" in directory
// pi@raspberrypi:/usr/share/arduino/hardware/arduino/avr
// goto lines with "compiler.c.extra_flags=" and "compiler.c.elf.extra_flags=" and add the following
// =-Wl,-u,vfprintf -lprintf_flt -lm -Wl,-u,vfscanf -lscanf_flt -lm
// then the printing will work fine

#include "encoders.hpp"
#include "interrupts.hpp"
#include "motors.hpp"
#include "control.hpp"
#include <stdio.h>

#define BUFFER_SIZE 10

char buffer[1024] = {0};

struct typedef robot_t{

	motor_t motor_power ;
	motor_t motor_direc ;

	encoder_t encoder_power ;
	encoder_t encoder_direc ;

	control_t control_direc_omega ;
	control_t control_direc_theta ;

	control_t cnotrol_power_omega ;
	control_t cnotrol_power_theta ;
	
} robot_t ;


static robot_t ;

static motor_t motor_power = init_motor( 12, 34, 35 );
static motor_t motor_direc = init_motor( 8, 37, 36 );

static encoder_t encoder_power = init_encoder( 18, 31 );
static encoder_t encoder_direc = init_encoder( 19, 38 );

static control_t control_direc_omega = init_control(10, 1) ;      // direction internal speed control
static control_t control_direc_theta = init_control( 2, 0) ;      // direction external position control 

static control_t control_power_omega = init_control( 5, 0.5) ;    // power internal speed control
static control_t control_power_theta = init_control( 1, 0) ;      // power external position control 

static float time_counter = 0;


// put your setup code here, to run once:
void setup() {
	
  	Serial.begin(115200);         // /dev/ttyUSB0   usb cable serial
  	Serial2.begin(115200);          // /dev/ttyS0     gpios on rpi
	
	pinMode(13, OUTPUT);            // blue LED

    start_encoders_interrupt();
    start_timer_interrupt();        // starts 10ms timer interrupt
    
	// print header
	sprintf(buffer, "Hello World! \n");
    Serial2.print(buffer);  
		
}


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

	
	
	char buffer_time_counter[BUFFER_SIZE] = {0} ;
	
	char buffer_set_point[BUFFER_SIZE] = {0} ;
    char buffer_feedback[BUFFER_SIZE] = {0} ;
    char buffer_power_omega[BUFFER_SIZE] = {0} ;
    char buffer_pid[BUFFER_SIZE] = {0} ;
    char buffer_odom[BUFFER_SIZE] = {0} ;
    char buffer_int[BUFFER_SIZE] = {0} ;            

    char buffer_direc_set_point[BUFFER_SIZE] = {0} ;
    char buffer_direc_feedback[BUFFER_SIZE] = {0} ;
    char buffer_direc_omega[BUFFER_SIZE] = {0} ;
    char buffer_direc_pid[BUFFER_SIZE] = {0} ;
    char buffer_direc_odom[BUFFER_SIZE] = {0} ;
    

	dtostrf( time_counter, 10, 3, buffer_time_counter );
	
    dtostrf( control_power_theta.set_point, 6, 3, buffer_set_point );
    dtostrf( encoder_power.theta, 6, 3, buffer_feedback );
    dtostrf( encoder_power.omega_mean, 6, 3, buffer_power_omega );
    dtostrf( control_power_omega.pid, 6, 3, buffer_pid );
    dtostrf( control_direc_omega.integrator, 6, 3, buffer_int );
    dtostrf( encoder_power.odom, 10, 1, buffer_odom );

	dtostrf( control_direc_theta.set_point, 6, 3, buffer_direc_set_point );
    dtostrf( encoder_direc.theta, 6, 3, buffer_direc_feedback );
    dtostrf( encoder_direc.omega_mean, 6, 3, buffer_direc_omega );
    dtostrf( control_direc_omega.pid, 6, 3, buffer_direc_pid );
    dtostrf( encoder_direc.odom, 10, 1, buffer_direc_odom );
    
	sprintf(buffer, " %s , %s , %s, %s, %s, %s, %s, %s, %s, %s, %s \n", 
		buffer_time_counter,
		buffer_set_point, buffer_feedback, buffer_pid, buffer_odom, buffer_power_omega,
		buffer_direc_set_point, buffer_direc_feedback, buffer_direc_pid, buffer_direc_odom, buffer_direc_omega
	);

	struct typedef robot_t{

		motor_t motor_power ;
		motor_t motor_direc ;
	
		encoder_t encoder_power ;
		encoder_t encoder_direc ;
	
		control_t control_direc_omega ;
		control_t control_direc_theta ;
	
		control_t control_power_omega ;
		control_t control_power_theta ;
		
	} robot_t ;
	
	robot_r mobile_robot = { motor_power, motor_direc, encoder_power, encoder_direc, 
		control_direc_omega, control_direc_theta, control_power_omega, control_power_theta } ;
	
	Serial2.print( mobile_robot );
	//Serial2.print( buffer );
	
	if(time_counter < 10.0){
		//Serial2.print( buffer );	
	}
	else{
		//time_counter = 0;
		
		//sprintf( buffer, "%s, \n", buffer_odom);    // TODO test without \n
		//Serial2.print( buffer );
		//time_counter = 0;	
		//mean_omega = 0;
		//integrator = 0;
		//direc.odom = 0;
	}

  	digitalWrite(13, HIGH);
  	delay(100);
	
}


//timer5 interrupt 10ms 
ISR(TIMER5_COMPA_vect){
		
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
