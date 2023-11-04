int led_state = 0;
char buffer[100] = {0};

float time_counter = 0;
float voltage = 0;

int32_t encoder_counter = 0;
int32_t old_encoder_counter = 0;
int32_t delta = 0;

float omega = 0;

char buffer_time[15] = {0};
char buffer_voltage[15] = {0};
char buffer_encoder[15] = {0};

// pins used for encoder 1
#define PORT1_NE1 31	// determines orientation B
#define PORT1_NE2 18	// accepts interrupts A

// pins used for encoder 2
#define PORT2_NE1 38	// determines orientation B
#define PORT2_NE2 19	// accepts interrupts A

// defines quadrature encoder pins
const byte A_Pin = PORT1_NE2;
const byte B_Pin = PORT1_NE1;

// pins used by the motor drivers
#define PWMB_1 12
#define BI1_1 35
#define BI2_1 34

#define PWMB_2 8
#define BI1_2 36
#define BI2_2 37

#define MAX_PWM 190     // maximum duty cycle for the PWM is 255/MAXPWM
#define VOLT_TO_PWM 255.0/12.0

#define DELAY_UP_DOWN 500
#define DELAY_LEFT_RIGHT 100

typedef struct motor_t {

  uint8_t pwm ;          // holds the pin number for the pwm
  uint8_t forward ;     // holds the pin number for moving forwards
  uint8_t backward ;    // holds the pin number for moving backwards
  
} motor_t ;

// functions declaration
int drive_voltage( motor_t motor, float voltage );
motor_t init_motor( uint8_t pwm, uint8_t forward, uint8_t backward );


// function to initialize the motor
motor_t init_motor( uint8_t pwm, uint8_t forward, uint8_t backward ){

  motor_t motor = { pwm, forward, backward } ;
  
  // motor pins mode 
  pinMode(motor.pwm, OUTPUT);
  pinMode(motor.forward, OUTPUT);
  pinMode(motor.backward, OUTPUT);
  
  drive_voltage(motor, 0);

  return motor;
}


// function to apply voltage to the motors
int drive_voltage( motor_t motor, float voltage ){

  if( voltage > 12.0 ) voltage = 12.0 ;
  else if(voltage < -12.0) voltage = -12.0 ;
    
  int pwm_value = abs(voltage) * VOLT_TO_PWM ;

  if(voltage < 0){
    digitalWrite( motor.forward, 0 );
    digitalWrite( motor.backward, 1 );
  }
  else{
    digitalWrite( motor.forward, 1 );
    digitalWrite( motor.backward, 0);
  }

  analogWrite( motor.pwm, pwm_value );
  
  return 0;
}

static motor_t powertrain = init_motor( 12, 34, 35 );
static motor_t direction = init_motor( 8, 37, 36 );

void setup() {
	// put your setup code here, to run once:
  	//Serial.begin(115200);       // /dev/ttyXXX    usb cable serial
  	Serial2.begin(115200);    // /dev/ttyS0     gpios on rpi
	
	pinMode(13, OUTPUT);

	drive_voltage( powertrain, 0.0);
  	drive_voltage( direction, 0.0);

	pinMode(A_Pin, INPUT_PULLUP);
	pinMode(B_Pin, INPUT_PULLUP);
	
	attachInterrupt(digitalPinToInterrupt(A_Pin), encoder_isr, RISING);
	
	cli();//stop interrupts

	//set timer4 interrupt at 1Hz
	TCCR4A = 0;					// set entire TCCR1A register to 0
	TCCR4B = 0;					// same for TCCR1B
	TCNT4  = 0;					//initialize counter value to 0

	// set compare match register for 10 ms increments
	OCR4A = 624;				// = (16*10^6) / (256) - 1 (must be <65536)

	// set compare match register for 50 ms increments
	//OCR4A = 3124;				// = (16*10^6) / (256) - 1 (must be <65536)
	
	
	// set compare match register for 1hz increments
	//OCR4A = 62499;				// = (16*10^6) / (256) - 1 (must be <65536)
	
	TCCR4B |= (1 << WGM12);		// turn on CTC mode

	TCCR4B |= (1 << CS12);  	// Set CS12 for 256 prescaler
	//TCCR4B |= (1 << CS12) | (1 << CS10);  	// Set CS12 and CS10 bits for 1024 prescaler
	
	TIMSK4 |= (1 << OCIE4A);	// enable timer compare interrupt

	sei();//allow interrupts
	
	// print header
	sprintf(buffer, "time [ms], voltage [V], speed [rad/s] \n");
    Serial2.print(buffer);  
	
	dtostrf( time_counter, 5, 3, buffer_time);
	dtostrf( voltage, 5, 3, buffer_voltage);
	dtostrf( encoder_counter, 5, 3, buffer_encoder);
	sprintf(buffer, "%s, %s, %s \n", buffer_time, buffer_voltage, buffer_encoder );
	Serial2.print(buffer);  
	
	

}

void loop() {
	// put your main code here, to run repeatedly:
	//Serial2.println("Hello");
	//delay(1000);

}

//timer1 interrupt 10ms 
ISR(TIMER4_COMPA_vect){
	time_counter += 0.010;	// adds 10 ms to counter	
		
  	if( time_counter < 3.0 ){
  		voltage = 0;
  	}
  	else if( time_counter < 6.0 ){
  		voltage = 12;
  	}
  	else if( time_counter < 9.0 ){
  		voltage = 0;
  	}
  	else if( time_counter < 12.0 ){
  		voltage = -12;
  	}
  	else if( time_counter < 15.0){
  		voltage = 0;
  	}
  	else{
  		voltage = 0;	
  	}

	drive_voltage( powertrain, voltage);

  	// toogles LED state
	if(led_state) led_state = 0;
	else led_state = 1;
	digitalWrite(13, led_state);

	delta = encoder_counter - old_encoder_counter ;
	old_encoder_counter = encoder_counter ;

	omega = (delta * 0.243912625 );		// converts from delta encoder to omega
	// supposes delta_t = 10ms
	// and gear ratio 2576
	
	dtostrf( time_counter, 6, 3, buffer_time);
	dtostrf( voltage, 6, 3, buffer_voltage);
	dtostrf( omega, 6, 3, buffer_encoder);
	sprintf(buffer, "%s, %s, %s \n", buffer_time, buffer_voltage, buffer_encoder );
	
  	if( time_counter < 15.0 ){
  		Serial2.print(buffer);
  	}
  	else{
  		Serial2.print("end \n");
  		//Serial.end();
  		Serial2.end();
  	}

}

void encoder_isr() {
	if( digitalRead( B_Pin ) ){		// checks for polarity
		encoder_counter++;	
		
	}
	else{
		encoder_counter--;	
	}

	
}
