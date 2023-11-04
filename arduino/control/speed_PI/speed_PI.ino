#include "encoders.hpp"
#include "interrupts.hpp"
#include "motors.hpp"

char buffer[100] = {0};

static motor_t powertrain = init_motor( 12, 34, 35 );
static motor_t direction = init_motor( 8, 37, 36 );

static encoder_t power = init_encoder( 18, 31 );
static encoder_t direc = init_encoder( 19, 38 );

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
    digitalWrite(13, HIGH);
    drive_voltage( powertrain, 9);
	delay(5000);
    digitalWrite(13, LOW);
    drive_voltage( powertrain, -9);
    delay(5000);
    
}


//timer4 interrupt 10ms 
ISR(TIMER4_COMPA_vect){
    char buffer_omega[15] = {0} ;
        
	//drive_voltage( powertrain, voltage);

  	// calculates instant speed
    calculate_speed(power);
    
	dtostrf( power.omega, 6, 3, buffer_omega );
    
	sprintf(buffer, "omega : [%s] \n", buffer_omega );	
  	Serial2.print(buffer);

}
