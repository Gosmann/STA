//#include "encoders.hpp"

#define PI 3.141592654

// function to initialize the encoders
encoder_t init_encoder( uint32_t A, uint32_t B ){

    encoder_t encoder = { A, B, 0, 0, 0, 0, 0} ;

    pinMode(encoder.A, INPUT_PULLUP);
    pinMode(encoder.B, INPUT_PULLUP);

    return encoder;
}

// calculates encoder speed
void calculate_speed( encoder_t * encoder ){
	
    int32_t delta = encoder->odom - encoder->old_odom ;
	
    // converts from delta encoder to omega [rad/s]
    if( encoder == &encoder_power){
        encoder->omega = ( (float)delta * 0.0609781 );  	//	( ( 2 * PI ) / ( 46 * 7 * 8 * 4 ) ) );   
    }
    else if( encoder == &encoder_direc){
      encoder->omega = ( (float)delta * 0.0290888 );		//	( ( 2 * PI ) / ( 75 * 9 * 8 * 4 ) ) );     
    }
    // supposes delta_t = 10ms
    // and gear ratio 2576

    // calculates moving average
    encoder->omega_mean = encoder->omega_mean * 0.90 + encoder->omega * 0.10 ;    
    
    encoder->old_odom = encoder->odom;
}

void calculate_pos( encoder_t * encoder){
		
    // converts from delta encoder to omega [rad/s]
    if( encoder == &encoder_power){
    	//encoder->theta = ( (float)encoder->odom * ( ( 2 * PI ) / ( 46 * 7 * 8 * 4 ) ) );     
        encoder->theta = ( (float)encoder->odom * 0.000609781 );     
    }
    else if( encoder == &encoder_direc){
    	//encoder->theta = ( (float)encoder->odom * ( ( 2 * PI ) / ( 75 * 9 * 8 * 4 ) ) );         	
    	encoder->theta = ( (float)encoder->odom * 0.000290888 ); 
    }
    
}

// treats encoder interrupt service routine
/*
void power_isr() {
    
    cli();    // stops interrupts
    
    if( digitalRead( encoder_power.B ) == digitalRead( encoder_power.A ) ){     // checks for polarity
        encoder_power.odom += 1.0000 ;
    }
    else{
        encoder_power.odom -= 1.0600 ;  
    }

    sei();    // starts interrupts
}

// treats encoder interrupt service routine
void direc_isr() {
  
    cli();    // stops interrupts
    
    if( digitalRead( encoder_direc.B ) == digitalRead( encoder_direc.A ) ){     // checks for polarity
        encoder_direc.odom += 1.0000 * 0.82 ;
    }
    else{
        encoder_direc.odom -= (1.0000 + 0.0018) * 0.82 ;  
    }

    sei();    //stop interrupts
}
*/
