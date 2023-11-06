#include "encoders.hpp"
#include "interrupts.hpp"
#include "motors.hpp"
#include "control.hpp"

char buffer[100] = {0};

static motor_t motor_power = init_motor( 12, 34, 35 );
static motor_t motor_direc = init_motor( 8, 37, 36 );

static encoder_t encoder_power = init_encoder( 18, 31 );
static encoder_t encoder_direc = init_encoder( 19, 38 );

static control_t control_direc_omega = init_control(15, 2) ;      // direction internal speed control
static control_t control_direc_theta = init_control( 2, 0) ;      // direction external position control 

static float time_counter = 0;

// put your setup code here, to run once:
void setup() {
	
  	//Serial.begin(115200);         // /dev/ttyUSB0   usb cable serial
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

	if( time_counter < 1.0 ){
		control_direc_theta.set_point = 0 ;
	}
	else if(time_counter < 5.0 ){
		control_direc_theta.set_point = 0.5 ;
	}
    /*
	else if(time_counter < 9.0 ){
		set_point_theta = 0 ;
	}
	else if(time_counter < 13.0 ){
		set_point_theta = -0.5 ;
	}
	else if(time_counter < 17.0 ){
		set_point_theta = 0 ;
	}
	else if(time_counter < 7.0 ){
		//set_point = -0.5 ;
	}
	else if(time_counter < 8.0 ){
		//set_point = 0 ;
	}
	else if(time_counter < 9.0 ){
		//set_point = 0.5 ;
	}
	else{
		//set_point_theta = 0.5 ;
	}
	*/
	
	char buffer_time_counter[15] = {0} ;
	char buffer_set_point[15] = {0} ;
    char buffer_feedback[15] = {0} ;
    char buffer_pid[15] = {0} ;
    char buffer_odom[15] = {0} ;
    char buffer_int[15] = {0} ;            

	dtostrf( time_counter, 6, 3, buffer_time_counter );
    dtostrf( control_direc_theta.set_point, 6, 3, buffer_set_point );
    dtostrf( encoder_direc.theta, 6, 3, buffer_feedback );
    dtostrf( control_direc_theta.pid, 6, 3, buffer_pid );
    dtostrf( control_direc_omega.integrator, 6, 3, buffer_int );
    dtostrf( encoder_direc.odom, 3, 0, buffer_odom );
    
	sprintf(buffer, "%s , %s , %s, %s \n", 
		buffer_time_counter, buffer_set_point, buffer_feedback, buffer_pid );

	//Serial2.print( buffer );
	
	if(time_counter < 10.0){
		Serial2.print( buffer );	
	}
	else{
		time_counter = 0;
		
		sprintf( buffer, "%s, \n", buffer_odom);    // TODO test without \n
		Serial2.print( buffer );
		//time_counter = 0;	
		//mean_omega = 0;
		//integrator = 0;
		//direc.odom = 0;
	}
  	
  	delay(10);
	
}


//timer5 interrupt 10ms 
ISR(TIMER5_COMPA_vect){
		
	time_counter += 0.010 ; 		// time_counter is in seconds

    {   // direc omega (speed) control
    
    	// calculates instant speed [rad/s]
        calculate_speed( &encoder_direc );
    
        // calculates PID
        calculate_pid( &control_direc_omega, encoder_direc.omega_mean) ;

        // updates motor voltage
        drive_voltage( motor_direc, control_direc_omega.pid );
    }

    {   // direc theta (position) control
    
        // calculates instant position [rad]
        calculate_pos(&encoder_direc);

        // calculates PID
        calculate_pid(&control_direc_theta, encoder_direc.theta ) ;

        // updates speed set point
        control_direc_omega.set_point = control_direc_theta.pid ;
    }        
    
}
