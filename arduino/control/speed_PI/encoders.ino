//#include "encoders.hpp"

// function to initialize the encoders
encoder_t init_encoder( uint8_t A, uint8_t B ){

    encoder_t encoder = { A, B, 0, 0 } ;
    
    pinMode(encoder.A, INPUT_PULLUP);
    pinMode(encoder.B, INPUT_PULLUP);

    return encoder;
}

// calculates encoder speed
void calculate_speed( encoder_t encoder){

    static int32_t old_odom = encoder.odom ;

    int32_t delta = encoder.odom - old_odom ;

    // converts from delta encoder to omega [rad/s]
    encoder.omega = ( (float)delta * 0.243912625 );         
    // supposes delta_t = 10ms
    // and gear ratio 2576
    
    old_odom = encoder.odom;
}


// treats encoder interrupt service routine
void power_isr() {
    
    if( digitalRead( power.B ) ){     // checks for polarity
        power.odom++;
    }
    else{
        power.odom--;  
    }
}

// treats encoder interrupt service routine
void direc_isr() {
    
    if( digitalRead( direc.B ) ){     // checks for polarity
        direc.odom++;
    }
    else{
        direc.odom--;  
    }
}
