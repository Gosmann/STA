
/*
  typedef struct control_t {

  float kp ;  
  float ki ;  
  float integrator ;

  float set_point ;
  //float old_set_point ;
  
  float feedback ;
  float error ;
  
  float pid ;
  
} control_t ;

*/

control_t init_control( float kp, float ki){

    control_t control = { kp, ki, 0, 0, 0, 0, 0 } ;

    return control ;
}

void calculate_pid( control_t * control, float feedback ){

    control->error = control->set_point - feedback;

    // anti wind-up
    if( abs( ( control->integrator + control->error ) * control->ki + control->error * control->kp ) < 12.0 ){
        control->integrator += control->error ;     // error * dt 
    }

    control->pid = control->kp * control->error + control->ki * control->integrator ;
    
}
