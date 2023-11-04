int led_state = 0;
char buffer[100] = {0};

float time_counter = 0;
float voltage = 0;
int32_t encoder_counter = 0;
float omega = 0;

char buffer_time[15] = {0};
char buffer_voltage[15] = {0};
char buffer_encoder[15] = {0};

void setup() {
	// put your setup code here, to run once:
  	//Serial.begin(115200);       // /dev/ttyXXX    usb cable serial
  	Serial2.begin(115200);    // /dev/ttyS0     gpios on rpi
	
	pinMode(13, OUTPUT);

	cli();//stop interrupts

	//set timer4 interrupt at 1Hz
	TCCR4A = 0;					// set entire TCCR1A register to 0
	TCCR4B = 0;					// same for TCCR1B
	TCNT4  = 0;					//initialize counter value to 0

	// set compare match register for 10 ms increments
	OCR4A = 624;				// = (16*10^6) / (256) - 1 (must be <65536)
	
	// set compare match register for 1hz increments
	//OCR4A = 62499;				// = (16*10^6) / (256) - 1 (must be <65536)
	
	TCCR4B |= (1 << WGM12);		// turn on CTC mode

	TCCR4B |= (1 << CS12);  	// Set CS12 for 256 prescaler
	//TCCR4B |= (1 << CS12) | (1 << CS10);  	// Set CS12 and CS10 bits for 1024 prescaler
	
	TIMSK4 |= (1 << OCIE4A);	// enable timer compare interrupt

	sei();//allow interrupts
	
	// print header
	sprintf(buffer, "time [ms], voltage [V], encoders [pulses] \n");
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
  		voltage = 5;
  	}
  	else if( time_counter < 9.0 ){
  		voltage = 0;
  	}
  	else if( time_counter < 12.0 ){
  		voltage = -5;
  	}
  	else if( time_counter < 15.0){
  		voltage = 0;
  	}
  	else{
  		voltage = 0;	
  	}
	

  	// toogles LED state
	if(led_state) led_state = 0;
	else led_state = 1;
	digitalWrite(13, led_state);

	dtostrf( time_counter, 6, 3, buffer_time);
	dtostrf( voltage, 6, 3, buffer_voltage);
	dtostrf( encoder_counter, 6, 3, buffer_encoder);
	sprintf(buffer, "%s, %s, %s \n", buffer_time, buffer_voltage, buffer_encoder );
	
  	if( time_counter <= 15.0 ){
  		Serial2.print(buffer);
  	}
  	else{
  		Serial.end();
  		Serial2.end();
  	}

}
