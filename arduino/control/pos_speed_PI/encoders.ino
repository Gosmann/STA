//#include "encoders.hpp"

// function to initialize the encoders
encoder_t init_encoder( uint8_t A, uint8_t B ){

    encoder_t encoder = { A, B, 0, 0, 0} ;
    
    pinMode(encoder.A, INPUT_PULLUP);
    pinMode(encoder.B, INPUT_PULLUP);

    return encoder;
}

// calculates encoder speed
void calculate_speed( encoder_t * encoder ){
	
    int32_t delta = encoder->odom - encoder->old_odom ;
	
    // converts from delta encoder to omega [rad/s]
    encoder->omega = ( (float)delta * 0.14959965 * 0.5 );         
    // supposes delta_t = 10ms
    // and gear ratio 2576
    
    encoder->old_odom = encoder->odom;
}

// treats encoder interrupt service routine
void power_isr() {
    //Serial2.println(" power isr ");
    cli();//stop interrupts
    
    if( digitalRead( power.B ) ){     // checks for polarity
        power.odom++;
    }
    else{
        power.odom--;  
    }

    sei();//stop interrupts
}

// treats encoder interrupt service routine
void direc_isr() {
    cli();//stop interrupts
    
    if( digitalRead( direc.B ) == digitalRead( direc.A ) ){     // checks for polarity
        direc.odom ++;
    }
    else{
        direc.odom -= 1.0185;  
        
    }

    sei();//stop interrupts
}
