// should be run with write_serial.c


// pins used by the motor drivers
#define PWMB_1 12
#define BI1_1 35
#define BI2_1 34

#define PWMB_2 8
#define BI1_2 37
#define BI2_2 36

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

  if( voltage > 9.0 ) voltage = 9.0 ;
  else if(voltage < -9.0) voltage = -9.0 ;
    
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

static motor_t powertrain = init_motor( 12, 35, 34 );
static motor_t direction = init_motor( 8, 37, 36 );

// put your setup code here, to run once:
void setup() {
  
  drive_voltage( powertrain, 0.0);
  drive_voltage( direction, 0.0);

  Serial2.begin(9600);    // /dev/ttyS0     on rpi
}

// put your main code here, to run repeatedly:
void loop() {

  while ( Serial2.available() == 0 ) {
    // do nothing
    digitalWrite(LED_BUILTIN, LOW);   
    drive_voltage( powertrain , 0.0);  
    drive_voltage( direction , 0.0);  
  }
  digitalWrite(LED_BUILTIN, HIGH);   
  
  char c = Serial2.read();
  if( c == 'U' || c == 'w' ){
    drive_voltage( powertrain, 9.0);
    delay(DELAY_UP_DOWN);   // define delay period
    //break;
  }
  else if( c == 'D' || c == 's' ){
    drive_voltage( powertrain, -9.0);  
    delay(DELAY_UP_DOWN);   
  }
  else if( c == 'L' || c == 'a' ){
    drive_voltage( direction, 2.0);  
    delay(DELAY_UP_DOWN);   
  }
  else if( c == 'R' || c == 'd' ){
    drive_voltage( direction, -2.0);  
    delay(DELAY_UP_DOWN);   
  }
  
  
  
  

}
