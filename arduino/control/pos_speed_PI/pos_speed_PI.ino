#include "encoders.hpp"
#include "interrupts.hpp"
#include "motors.hpp"

char buffer[100] = {0};

static motor_t powertrain = init_motor( 12, 34, 35 );
static motor_t direction = init_motor( 8, 37, 36 );

static encoder_t power = init_encoder( 18, 31 );
static encoder_t direc = init_encoder( 19, 38 );

static float error = 0 ;
static float time_counter = 0 ;
static float set_point = 0 ;
static float pid = 0 ;
static float integrator = 0;

static float mean_omega = 0;
static float set_point_theta = 0;

static int count_over = 0;

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
		//set_point_theta = -0.05 ;
		set_point_theta = 0 ;
	}
	else if(time_counter < 5.0 ){
		set_point_theta = 0.50 ;
		//set_point_theta = -0.50 ;
	}
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
	
	
	char buffer_time_counter[15] = {0} ;
	char buffer_set_point[15] = {0} ;
    char buffer_omega[15] = {0} ;
    char buffer_pid[15] = {0} ;
    char buffer_odom[15] = {0} ;
    char buffer_int[15] = {0} ;            

	dtostrf( time_counter, 6, 3, buffer_time_counter );
    dtostrf( set_point_theta, 6, 3, buffer_set_point );
    dtostrf( direc.odom * 0.000747998  , 6, 3, buffer_omega );
    dtostrf( pid, 6, 3, buffer_pid );
    dtostrf( integrator, 6, 3, buffer_int );
    dtostrf( direc.odom, 3, 0, buffer_odom );

	//sprintf(buffer, "[%s] : [%s] [%s] : [%s] [%s] [%s] \n", 
	//	buffer_time_counter, buffer_set_point, buffer_omega, buffer_pid, buffer_int, buffer_odom);

	sprintf(buffer, "%s , %s , %s, %s \n", 
		buffer_time_counter, buffer_set_point, buffer_omega, buffer_pid );

	//Serial2.print( buffer );
	
	if(time_counter < 22.0){
		//Serial2.print( buffer );	
	}
	else{
		time_counter = 0;
		
		sprintf( buffer, "%s, ", buffer_odom);
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
	
	

	// calculates instant speed
    calculate_speed(&direc);
	
	mean_omega = mean_omega * 0.90 + direc.omega * 0.10 ;

	error = set_point - mean_omega ;

	float kp = 15 ;
	float ki = 2 ;

	static float old_set_point = set_point ;

	if( set_point != old_set_point){
		//integrator = 0 ;
		old_set_point = set_point ;
	}

	// anti wind-up
	if( abs( ( integrator + error ) * ki + error * kp ) < 12.0 ){
		integrator += error ; // error * dt	
	}
	
	pid = error * kp + integrator * ki ;

	drive_voltage( direction, pid );

	static float old_set_point_theta = set_point_theta ;
	    
	count_over++;
	if(count_over >= 1 || set_point_theta != old_set_point_theta ){

		old_set_point_theta = set_point_theta ;
		count_over = 0;

		float error_theta = set_point_theta - direc.odom * 0.000747998 ;

		/*
		if( error_theta >= 0 ){
			error_theta = set_point_theta - direc.odom * 0.000747998 ;	
		}
		else{
			error_theta = set_point_theta - direc.odom * 0.000747998 * 0.85 ;
		}
		*/

		

		if( abs(error_theta) < 0.001)
			error_theta = 0;

		float kp_ext = 2;
		float ki_ext = 0.0;

		static float integrator_ext = 0;

		integrator_ext += error_theta;
	
		float pid_ext = kp_ext * error_theta + ki_ext * integrator_ext;

		set_point = pid_ext;		
	}

    

}
