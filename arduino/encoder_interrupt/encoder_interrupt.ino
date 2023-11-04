// pins used for encoder 1
#define PORT1_NE1 31	// determines orientation B
#define PORT1_NE2 18	// accepts interrupts A

// pins used for encoder 2
#define PORT2_NE1 38	// determines orientation B
#define PORT2_NE2 19	// accepts interrupts A


const byte led_pin = 13;
int led_state = 0;

// defines quadrature encoder pins
const byte A_Pin = PORT2_NE2;
const byte B_Pin = PORT2_NE1;

int32_t counter = 0;	// defines signed int


void setup() {
	Serial.begin(115200);    // /dev/ttyUSB0 on rpi
	
	pinMode(led_pin, OUTPUT);
	
	pinMode(A_Pin, INPUT_PULLUP);
	pinMode(B_Pin, INPUT_PULLUP);
	
	attachInterrupt(digitalPinToInterrupt(A_Pin), encoder_isr, RISING);
}

void loop() {
		
	delay(1000);
	
	Serial.println(counter);
}

void encoder_isr() {

	if( digitalRead( B_Pin ) ){		// checks for polarity
		counter++;	
		
	}
	else{
		counter--;	
	}

	if( led_state ) led_state = 0;
	else led_state = 1;
	
	digitalWrite(led_pin, led_state);
}
