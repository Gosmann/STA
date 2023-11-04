// pins used for encoder 1 (powertrain motor encoder)
#define PORT1_NE1 31    // determines orientation B
#define PORT1_NE2 18    // accepts interrupts A

// pins used for encoder 2 (direction motor encoder)
#define PORT2_NE1 38    // determines orientation B
#define PORT2_NE2 19    // accepts interrupts A

// defines quadrature encoder pins
//uint8_t A_Pin = PORT1_NE2;
//uint8_t B_Pin = PORT1_NE1;

typedef struct encoder_t {

  uint8_t A ;           // holds the pin number for the A quadrature signal
  uint8_t B ;           // holds the pin number for the B quadrature signal
  
  int32_t odom ;        // holds total distance ever travelled in encoder ticks
  int32_t old_odom ;

  float omega ;         // holds the instant angular velocity of the wheels
  
} encoder_t ;

// function to initialize the encoders
encoder_t init_encoder( uint8_t A, uint8_t B );
