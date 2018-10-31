//motor configuration
//                ^N
//            M3 ^     M1 v
//            M4 v     M2 ^
int M3_PIN = 5;
int M1_PIN = 2;
int M4_PIN = 6;
int M2_PIN = 3;

float M1,M2,M3,M4;
float Mout1,Mout2,Mout3,Mout4;

//lag filter motor/
float M3f = 1000;
float M3old = 1000;
float M3old2 = 1000;
float M1f = 1000;
float M1old = 1000;
float M1old2 = 1000;
float M4f = 1000;
float M4old = 1000;
float M4old2 = 1000;
float M2f = 1000;
float M2old = 1000;
float M2old2 = 1000;
#define MINCOMMAND 900
#define MIDCOMMAND 1500
#define MAXCOMMAND 2000



//motor command
// void motor_initialize() 
// {
//   pinMode(M3_PIN,OUTPUT);  
//   pinMode(M1_PIN,OUTPUT); 
//   pinMode(M4_PIN,OUTPUT); 
//   pinMode(M2_PIN,OUTPUT); 
// }

static void motor_command() 
{
  Mout1=M1/8;
  Mout2=M2/8;
  Mout3=M3/8;
  Mout4=M4/8;
  sayend(Mout1);
}

static void motor_command_all(int Values) 
{
  Mout1=Values/8;
  Mout2=Values/8;
  Mout3=Values/8;
  Mout4=Values/8;

}
// static void motor_Lag(){
//   ////////lag lead compensator filter motor////////////////////////////////////
//      float k_Lag = 5.95;//3.95 5.85 1.85 0.45 1.078 1.0 1.25
//      float k_Lead = 25.5;//35.25
//      float diff_M3 = (M3 - M3old2)/0.02;
//      float temp_M3 = diff_M3 + (M3 - M3f)*k_Lead;//35.5
//      M3f = M3f + (temp_M3*G_Dt*k_Lag);
     
//      float diff_M1 = (M1 - M1old2)/0.02;
//      float temp_M1 = diff_M1 + (M1 - M1f)*k_Lead;//35.5
//      M1f = M1f + (temp_M1*G_Dt*k_Lag);
                        
//      float diff_M4 = (M4 - M4old2)/0.02;
//      float temp_M4 = diff_M4 + (M4 - M4f)*k_Lead;//35.5
//      M4f = M4f + (temp_M4*G_Dt*k_Lag);
     
//      float diff_M2 = (M2 - M2old2)/0.02;
//      float temp_M2 = diff_M2 + (M2 - M2f)*k_Lead;//35.5
//      M2f = M2f + (temp_M2*G_Dt*k_Lag);
     
//      M3old2 = M3old;
//      M3old = M3;//store PWM for next diff
//      M1old2 = M1old;
//      M1old = M1;
//      M4old2 = M4old;
//      M4old = M4;
//      M2old2 = M2old;
//      M2old = M2;
// }
static void tomotor()
{
        // Protect allocate 
        if (ahrs_r > PROTECT_PROP_ANGLE || ahrs_r <-PROTECT_PROP_ANGLE || ahrs_p >PROTECT_PROP_ANGLE || ahrs_p <-PROTECT_PROP_ANGLE ) armed =0;
        
        if (armed) 
        {
//           motor_Lag();
           // if(!flag_trim) digitalWrite(GREEN_LED, HIGH);
            //x type  (^ N)
            //CH_THR=constrain(CH_THR,1000,1300); //for test
            //multiply *0.7071  with roll and pitch when tune with positive configuration
            M3 = constrain(control_to + control_pitch + control_roll - control_yaw, 1100, 1900);   //ccw
                                                                                                  M1 = constrain(control_to + control_pitch - control_roll + control_yaw, 1100, 1900);  //cw
            M4 = constrain(control_to - control_pitch + control_roll + control_yaw, 1100, 1900);   //cw
                                                                                                  M2 = constrain(control_to - control_pitch - control_roll - control_yaw, 1100, 1900);  //ccw  
            motor_command(); 
//            if(FLIGHT_MODE(AUX_1) == 1) {
//                if (CH_THR < 1150) 
//                  {
//                    control_to = map(CH_THR,1050,1150,1150,1350);
//                  }
//             }
//            M3 =  constrain(control_to+control_pitch  -  control_yaw, 1150, 1900);
//            M2 =  constrain(control_to-control_pitch  -  control_yaw, 1150, 1900);
//            M4 =  constrain(control_to+control_roll   +  control_yaw, 1150, 1900);       
//            M1=  constrain(control_to-control_roll   +  control_yaw, 1150, 1900);


    
        }
        
        
        if (FLIGHT_MODE_CHECK == STABILIZE_MODE) {
          if (CH_THR < CH_THR_threshold) 
            {
              Mout1 = MINCOMMAND/8;
              Mout2 = MINCOMMAND/8;
              Mout3 = MINCOMMAND/8;
              Mout4 = MINCOMMAND/8; 
            }
        }
//        else if(FLIGHT_MODE(AUX_1) == 1) {
//          if (CH_THR < CH_THR_threshold) 
//            {
//              M3 = 1150;
//              M1 = 1150;
//              M4 = 1150;
//              M2 = 1150;
//            }
//        }
        
//            roll_I_rate=0;  
//            pitch_I_rate=0;
//            yaw_I_rate=0;
            
//            roll_I_level=0;
//            pitch_I_level=0;
//            to_I=0;  
        
        // sayend("motor_command");
         
}
static void ARM_DECISION()
{
          if (CH_THR < CH_THR_threshold) 
        {
          
          // Arm-------------------------------------------------------------------------------------
            if (RF_YAW > 1800 && armed == 0) 
            {
              count_time++;
              if(count_time>=TIME_ARM){
                armed = 1;
                count_time=0; //reset count_time
                //alt_trim=getAltitude;
                //prepare parameter for next take off
                flag_takeoff_done=false;
                arming_trim();
                //check motor
                motor_command_all(MINCOMMAND+150);
                sayend("ARMED");
                delay(500);
                motor_command_all(MINCOMMAND);
              }
            }else if(RF_YAW >1100 && RF_YAW<1800) count_time=0; //reset count_time  
          // Disarm-----------------------------------------------------------------------------------
            else if (RF_YAW < 1100 && armed == 1) 
            {     
              count_time++;    
                 if(count_time>=TIME_ARM*0.3){
                    armed = 0;
                    motor_command_all(MINCOMMAND);
                    
                    sayend("DISARMED");
                    disarming_trim();
                    //-----------------------------------
                    //reset all PID param except yaw for next take off
                    roll_I_rate=0;  
                    pitch_I_rate=0;
//                   yaw_I_rate=0;
                    roll_I_level=0;
                    pitch_I_level=0;
                    
                }
            }
        } 
        
        if (armed == 0) 
        {
            Mout1 = MINCOMMAND/8;
            Mout2 = MINCOMMAND/8;
            Mout3 = MINCOMMAND/8;
            Mout4 = MINCOMMAND/8;  
            
             // if(!flag_trim) digitalWrite(GREEN_LED, LOW);
            
            roll_I_rate=0;  
            pitch_I_rate=0;
            yaw_I_rate=0;
            roll_I_level=0;
            pitch_I_level=0;
        }
}
//static void Land_check() {
//  if(armed){
//    if ( M1 < LAND_CK_MOTOR && M2 < LAND_CK_MOTOR && M3 < LAND_CK_MOTOR && M4 < LAND_CK_MOTOR) {
//      Landed = true;
//    }else {
//      Landed = false;
//    }
//  }else{ Landed = true; }
//}
