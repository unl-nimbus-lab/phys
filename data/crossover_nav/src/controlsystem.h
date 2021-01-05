//#include "DerivativeFilter.h"
//
//DerivativeFilter<float,9> derivative_roll;
//DerivativeFilter<float,9> derivative_pitch;
float hold_yaw=0;
float pitch_desire=0;
float roll_desire=0;
float I_vel_roll=0;
float I_vel_pitch=0;
bool initialize_vp_hold=false;
bool initialize_p_hold = false;
//auto trim
static void CONTROLSYSTEM()
{
  //make small rotate when sonar on   // not fix yet  but ok
  if(control_to==0) factor_rotate = 1;
  else              factor_rotate = 0.5;

//PID Control STABLE        
///////////////////////////////////////////////////////////////////////////////////////////////////////////////
            
            
            //DCM  mpu6050
            // YAW CONTROL
            RF_YAW_PLUS_kep=(RF_YAW - yaw_mid);
            pitch_desire = RF_PIT - pitch_mid;
            roll_desire = RF_ROLL - roll_mid;
            
            applyDeadband(pitch_desire,30);
            applyDeadband(roll_desire,30);
            applyDeadband(RF_YAW_PLUS_kep,20);


//      pitch_desire = (RF_PIT - pitch_mid)*(0.0015*factor_rotate);
      if(FLIGHT_MODE_CHECK == VEL_HOLD) {
        if(!initialize_vp_hold) {
          setup_home_position();
          initialize_vp_hold = true;
        }
        if(roll_desire==0 && pitch_desire==0) {
          if(!initialize_p_hold){
            des_hx = hx; //hx
            des_hy = hy;
            initialize_p_hold=true;
          }
          float delta_x = (hx-des_hx);  //hx
          float delta_y = (hy-des_hy);
          Dhx_b= -delta_x*cos_yaw-delta_y*sin_yaw;
          Dhy_b=  delta_x*sin_yaw-delta_y*cos_yaw;
          
          roll_desire = Dhx_b * Kp_pos * 3.5;  //350*0.01
          pitch_desire = -Dhy_b * Kp_pos * 3.5;
        }else{
          initialize_p_hold = false;
        }
        float error_roll_vel = (roll_desire*0.002-(vx_theory>0.001 || vx_theory<-0.001 ? vx_theory:0));   //0.002 = max vel => +- 0.8 m/s
        I_vel_roll += Ki_vel*error_roll_vel*G_Dt;
        I_vel_roll  = constrain(I_vel_roll,-0.2,0.2);
        roll_desire = Kp_vel*error_roll_vel + I_vel_roll;
        roll_desire = constrain(roll_desire,-0.18,0.18); //+- 10 degree constrain
        
        float error_pitch_vel = (pitch_desire*0.002-(-vy_theory>0.001 || -vy_theory<-0.001 ? -vy_theory:0));   //0.002 = max vel => +- 0.8 m/s
        I_vel_pitch += Ki_vel*error_pitch_vel*G_Dt;
        I_vel_pitch  = constrain(I_vel_pitch,-0.2,0.2);
        pitch_desire = Kp_vel*error_pitch_vel + I_vel_pitch;
        pitch_desire = constrain(pitch_desire,-0.18,0.18); //+- 10 degree constrain
//        pitch_desire = pitch_desire*(0.0015*factor_rotate);
      }else {
        initialize_p_hold = false;
        initialize_vp_hold = false;
        roll_desire = roll_desire*(0.0015*factor_rotate);
        pitch_desire = pitch_desire*(0.0015*factor_rotate);
      }
            //pitch_desire   += vp*
            //roll_desire    += vr*2;

//if(RF_YAW_PLUS_kep==0)
//{
//
//            
//          //sliding mode test v.1  non-filter
////          smc(hold_yaw);
////          control_yaw = u;
//          
//          
//          
//          
////            err_yaw_level = wrap_pi(hold_yaw - ahrs_y);
////            err_yaw_rate = (err_yaw_level*2) - (gyro[ZAXIS]);  
////            yaw_I_rate += err_yaw_rate*G_Dt*100;     
////            yaw_I_rate = constrain(yaw_I_rate, -100, 100);
////            yaw_D_rate = (-gyro[ZAXIS] - err_yaw_ant_rate)/(G_Dt*100);    
////            err_yaw_ant_rate = (-gyro[ZAXIS]);      
////            control_yaw = (Kp_rateYaw*err_yaw_rate) + (/*Ki_rateYaw**/yaw_I_rate) + (Kd_rateYaw*yaw_D_rate)+110;
//            
//            
//            
////            saycomma(hold_yaw);
////            saycomma(ahrs_y);
////            sayend(control_yaw);
//            
//}
//else
//{
//
//            
//            err_yaw_rate = (-(RF_YAW_PLUS_kep*(2.5 * 0.002))) - (gyro[ZAXIS]);  
//            yaw_I_rate += err_yaw_rate*G_Dt;     
//            yaw_I_rate = constrain(yaw_I_rate, -200, 200);
//            yaw_D_rate = (-gyro[ZAXIS] - err_yaw_ant_rate)/(G_Dt*100);    
//            err_yaw_ant_rate = (-gyro[ZAXIS]);      
//            control_yaw = (Kp_rateYaw*err_yaw_rate) + (Ki_rateYaw*yaw_I_rate) + (Kd_rateYaw*yaw_D_rate); 
////            sayend(control_yaw);
//            hold_yaw = ahrs_y;
////  sayend("move");
//}

            //----------------------------------------------------------------------------------------------------
            // ROLL CONTROL--------------------------------------------------------------------------------------
            err_roll_level = Kp_levelRoll*(roll_desire-ahrs_r) - (gyro[XAXIS]);
            roll_I_level += err_roll_level*G_Dt*100; 
            roll_I_level = constrain(roll_I_level, -100, 100); 
            //roll_D_level = (tar*roll_D_level/(tar+G_Dt))+((err_roll_level - err_roll_ant_level)/(tar+G_Dt)); 
            //err_roll_ant_level = err_roll_level;       
//            saytab(roll_I_level);
            
            
            roll_D_level = (err_roll_level - last_input_roll) / G_Dt;
            roll_D_level = err_roll_ant_level + (G_Dt/(tar+G_Dt))*(roll_D_level-err_roll_ant_level);
            last_input_roll = err_roll_level;
            err_roll_ant_level = roll_D_level;    

//            derivative_roll.update(err_roll_level, currentTime);
//            roll_D_level = derivative_roll.slope() * 1.0e6f;
 
            control_roll = Kp_rateRoll*err_roll_level + Ki_rateRoll*roll_I_level + Kd_rateRoll*roll_D_level; 
            //----------------------------------------------------------------------------------------------------
            
            // PITCH CONTROL-------------------------------------------------------------------------------------
            err_pitch_level = Kp_levelPitch*(pitch_desire-(-ahrs_p)) - (-gyro[YAXIS]);   
            pitch_I_level += err_pitch_level*G_Dt*100;  
            pitch_I_level = constrain(pitch_I_level, -100, 100);
            //pitch_D_level = (tar*pitch_D_level/(tar+G_Dt))+((err_pitch_level - err_pitch_ant_level)/(tar+G_Dt));       
            //err_pitch_ant_level = err_pitch_level;   
//            sayend(roll_I_level);
            
            
            pitch_D_level = (err_pitch_level - last_input_pitch) / G_Dt;
            pitch_D_level = err_pitch_ant_level + (G_Dt/(tar+G_Dt))*(pitch_D_level-err_pitch_ant_level);
            last_input_pitch = err_pitch_level;
            err_pitch_ant_level = pitch_D_level; 
//            derivative_pitch.update(err_pitch_level, currentTime);
//            pitch_D_level = derivative_pitch.slope() * 1.0e6f;

            control_pitch = Kp_ratePitch*err_pitch_level + Ki_ratePitch*pitch_I_level + Kd_ratePitch*pitch_D_level;   
            
            //----------------------------------------------------------------------------------------------------

            // YAW CONTROL  
            RF_YAW_PLUS_kep=(RF_YAW - yaw_mid);
            

              applyDeadband(RF_YAW_PLUS_kep,15);
              
              if(!AUTO_MODE) {
                if(RF_YAW_PLUS_kep!=0) {
                  if(armed && (CH_THR > CH_THR_threshold ) ) {
                    err_yaw_level = -RF_YAW_PLUS_kep*YAW_TURN_RATE;   //prevent arm action to incress this value
                  }
                  RF_YAW_PLUS   =  ahrs_y;
                }else{
                   err_yaw_level = Kp_levelYaw*wrap_pi(RF_YAW_PLUS - ahrs_y);
                }
              }else{
                    if((wrap_pi(desire_yaw - ahrs_y))> SPAN_OF_YAW_TURNING) {
                      err_yaw_level = -400*YAW_TURN_RATE_AUTO;   //turn clockwise ... assuming
                    }else if((wrap_pi(desire_yaw - ahrs_y))< -SPAN_OF_YAW_TURNING) {
                      err_yaw_level = 400*YAW_TURN_RATE_AUTO;   //turn counter 
                    }else{
                      err_yaw_level = Kp_levelYaw*wrap_pi(desire_yaw - ahrs_y);
                      RF_YAW_PLUS   =  desire_yaw;
                    }
              }
                

            
            err_yaw_rate = (err_yaw_level*2) - (gyro[ZAXIS]);  
            
            yaw_I_rate += Ki_rateYaw*err_yaw_rate*G_Dt;     
            yaw_I_rate = constrain(yaw_I_rate, -YAW_I_MAX, YAW_I_MAX);
            
            yaw_D_rate = (err_yaw_rate - last_input_yaw) / G_Dt;
            yaw_D_rate = err_yaw_ant_rate + (G_Dt/(tar+G_Dt))*(yaw_D_rate-err_yaw_ant_rate);
            last_input_yaw = err_yaw_rate;
            err_yaw_ant_rate = yaw_D_rate; 
            
            control_yaw = (Kp_rateYaw*err_yaw_rate) + (yaw_I_rate+YAW_I_OFFSET) + (Kd_rateYaw*yaw_D_rate); 
            control_yaw = constrain(control_yaw,-250,250);


        
        
        
        //stable 
        if          (FLIGHT_MODE_CHECK == STABILIZE_MODE){
                            altitude_holding_off();
                            //for smooth accent and descent 
                            control_to=CH_THR+((g3)*(-200));
        }else if    (FLIGHT_MODE_CHECK == ALT_MODE || FLIGHT_MODE_CHECK == TAKE_OFF || FLIGHT_MODE_CHECK==VEL_HOLD){
                            //-------------------------------------------------------------------------------------
                            //use body frame  100hz
//                            err_acc_climb_rate = acc_z_desire - (g3 - 0.085);
//                                
//                            acc_climb_rate_I+=KI_alt_acc * err_acc_climb_rate;
//                            acc_climb_rate_I=constrain(acc_climb_rate_I,-100,100); 
//                            
//                            acc_climb_rate_dot = (err_acc_climb_rate - acc_climb_rate_Input_last) / G_Dt;
//                            acc_climb_rate_dot = acc_climb_rate_dot_last + (G_Dt/(0.1+G_Dt))*(acc_climb_rate_dot-acc_climb_rate_dot_last);
//                            acc_climb_rate_Input_last = err_acc_climb_rate;
//                            acc_climb_rate_dot_last = acc_climb_rate_dot;    
//                                
//                            
//                            alt_control = KP_alt_acc * err_acc_climb_rate + acc_climb_rate_I + KD_alt_acc * acc_climb_rate_dot;
//                            //old acc_z up = +   , downing = -
//                            control_to=CH_THR + constrain(alt_control,-300,300);  // - = control up     ,   + = control down; 
//                                saycomma(err_alt_level);
//                                saycomma(alt_control);
//                                sayend(control_to);
   
                              
                              
                              z_up = (CH_THR-THR_MID);
                              
                              if(FLIGHT_MODE_CHECK==TAKE_OFF){
                                z_up = 900;   //maybe change to max throttle - mid throttle
                                if(!flag_takeoff_processing) { /*desire_alt = estimated_altitude;*/ takeoff_height = desire_alt + 1.0; flag_takeoff_processing=true; }
                              }
                              applyDeadband(z_up,100);
                              desire_alt +=z_up*0.00001;          //incressing rate (slow 0.000005)  ********************************************************
                              if(FLIGHT_MODE_CHECK==TAKE_OFF){
                                if(desire_alt>takeoff_height) {
                                  desire_alt = takeoff_height;
                                  if(estimated_altitude>0.8*desire_alt){
                                      FLIGHT_MODE_CHECK==ALT_MODE;
                                      flag_takeoff_done=true;
                                      flag_takeoff_processing=false;
                                  }
                                }
                                
                              }
                              float err_hz = desire_alt - estimated_altitude;
                              float surface_alt = 0.1*estimated_velocity - (lambda_alt*err_hz);//surface
                              hz_I = hz_I + (Ki_altitude*surface_alt*G_Dt);
                              hz_I = constrain(hz_I, -300, 300);
                              float uthrottle2 = err_hz*Kp_altitude - hz_I - 0.1*estimated_velocity*Kd_altitude - (g3 - 0.085)*Ka_altitude;
      
                              control_to = HOVERING_THROTTLE_now + uthrottle2 * QUAD_MASS;
                              

        }

//            
//        if(AUX_1>1600){
//            // landing test
//            
//        }


      
      

}
