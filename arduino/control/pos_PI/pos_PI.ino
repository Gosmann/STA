#include "encoders.hpp"
#include "interrupts.hpp"
#include "motors.hpp"

char buffer[100] = {0};

static motor_t powertrain = init_motor( 12, 34, 35 );
static motor_t direction = init_motor( 8, 37, 36 );

static encoder_t power = init_encoder( 18, 31 );
static encoder_t direc = init_encoder( 19, 38 );

static float set_point = 0 ;

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
    
  // shows sign of life  
	float time_counter = (float)millis() * 0.001 ;	// timer_counter is in seconds

	if( time_counter < 2.0 ){
		set_point = 0 ;
	}
	else if(time_counter < 4.0 ){
		set_point = 3.1415 ;
	}
	else if(time_counter < 6.0 ){
		set_point = 0 ;
	}
	else{
		set_point = 0 ;
	}
	
	
}


//timer4 interrupt 10ms 
ISR(TIMER5_COMPA_vect){

	float time_counter = (float)millis() * 0.001 ; 		// time_counter is in seconds

	char buffer_time_counter[15] = {0} ;
	char buffer_set_point[15] = {0} ;
    char buffer_omega[15] = {0} ;
    char buffer_pid[15] = {0} ;
    char buffer_odom[15] = {0} ;
            
  	// calculates instant speed
    //calculate_speed(&power);
    float theta = direc.odom * 0.010471976 ;

	float error = set_point - theta ;

	float kp = 20 ;
	float ki = 0.2 ;

	static float integrator = 0;

	// anti wind-up
	if( abs( ( integrator + error * 0.010 ) * ki + error * kp ) < 12.0 ){
		integrator += error * 0.010 ; // error * dt	
	}
	
	float pid = error * kp + integrator * ki ;

	drive_voltage( direction, pid );

    dtostrf( time_counter, 6, 3, buffer_time_counter );
    dtostrf( set_point, 6, 3, buffer_set_point );
    dtostrf( theta , 6, 3, buffer_omega );
    dtostrf( pid, 6, 3, buffer_pid );
    dtostrf( direc.odom, 6, 3, buffer_odom );

	sprintf(buffer, "[%s] : [%s] [%s] : [%s] [%s] \n", 
		buffer_time_counter, buffer_set_point, buffer_omega, buffer_pid, buffer_odom);
		
  	Serial2.print( buffer );

}
