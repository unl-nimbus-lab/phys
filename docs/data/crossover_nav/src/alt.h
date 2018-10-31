float FACTOR = 1;
float KP1 = 0.55*FACTOR; // PI observer velocity gain 
float KP2 = 1.0*FACTOR;  // PI observer position gain
float KI = 0.001/FACTOR; // PI observer integral gain (bias cancellation)
bool initialized =false;
float   altitude_error_i = 0;
float   acc_scale  = 0.0;
float   altitude_error = 0;
float   inst_acceleration = 0.0;
float   delta = 0;
float   estimated_velocity = 0.0;
float   estimated_altitude = 0.0;

float   last_orig_altitude = 0;
float   last_estimated_altitude = 0;



//static void compute_compensated_acc()
//{
//         //get rid of gravity (only acc on axis affect will be probe)
//         g1 = 2*(q1*q3 - q0*q2);
//         g2 = 2*(q0*q1 + q2*q3);
//         g3 = q0*q0 - q1*q1 - q2*q2 + q3*q3;
//}
//
//static void compute_dynamic_acceleration_vector() {
//  //prepare to use q_conjugate
//  float qc0=q0;
//  float qc1=-q1;
//  float qc2=-q2;
//  float qc3=-q3;
//  
//  
//  
//    //q * q_compensate(0,g1,g2,g3)
//    float x1=q1;
//    float y1=q2;
//    float z1=q3;
//    float w1=q0;
//    //acc
//    float x2=g1;
//    float y2=g2;
//    float z2=g3;
//    float w2=0;
//            w = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2;
//            x = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2;
//            y = w1 * y2 + y1 * w2 + z1 * x2 - x1 * z2;
//            z = w1 * z2 + z1 * w2 + x1 * y2 - y1 * x2;
//    //(q*q_compensate) * q_c
//    x1=x;
//    y1=y;
//    z1=z;
//    w1=w;
//    
//    //compute_compensated_acc
//    x2=qc1;
//    y2=qc2;
//    z2=qc3;
//    w2=qc0;
//
//            w = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2;
//            x = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2;
//            y = w1 * y2 + y1 * w2 + z1 * x2 - x1 * z2;
//            z = w1 * z2 + z1 * w2 + x1 * y2 - y1 * x2;
//}

static float compute_altitude(float dt,float compensated_acceleration,float altitude)
{

        // Initialization
            if (!initialized) {
              initialized = true;
              estimated_altitude = altitude ;
              // saytab("test init altitude");saytab(alt_-alt_trim);sayend(altitude);
              estimated_velocity = 0;
              altitude_error_i = 0;
            }
            
        // Estimation Error
         altitude_error   =  altitude -  estimated_altitude;
         altitude_error_i =  altitude_error_i +  altitude_error;
         altitude_error_i =  constrain(altitude_error_i,-150,150);  //changed from -2500 ,2500
         
//         saycomma(altitude);
//         saycomma(estimated_altitude);
//         sayend(altitude_error_i);
         inst_acceleration = compensated_acceleration * 9.80665 +  altitude_error_i *  KI;

        // Integrators
         delta =  inst_acceleration * dt + ( KP1 * dt) *  altitude_error;
         estimated_altitude += ( estimated_velocity/5.0 +  delta) * (dt / 2) + ( KP2 * dt) *  altitude_error;
         estimated_velocity +=  delta*10.0 ;
         
        return  estimated_altitude;
}


//float ax ;
//float ay ;

//float acc_ef_y;
//float acc_ef_x;

static void update(float acc_x,float acc_y,float acc_z,float alt,float dt) 
{
  
  if(!(alt!=alt)){
//    compute_compensated_acc();  //q*acc  -> g
//get rid of gravity (only acc on axis affect will be probe)
      
         g1 = 2*(q1*q3 - q0*q2);
         g2 = 2*(q0*q1 + q2*q3);
         g3 = q0*q0 - q1*q1 - q2*q2 + q3*q3;

      //accel_without_gravity_body_frame
      g1=acc_x-g1;
      g2=acc_y-g2;
      g3=acc_z-g3/* - g3_trim*/;

    // Update_trig();
//    vx_theory+=(ONE_G*(_sin((ahrs_r>0.025 || ahrs_r<-0.025 ? ahrs_r : 0))/*-g2*/)-kdrag*vx_theory)*dt/**0.667*/;  //try /1.5=*0.667 divine
//    vy_theory+=(ONE_G*(_cos((ahrs_r>0.025 || ahrs_r<-0.025 ? ahrs_r : 0))*_sin((ahrs_p>0.025 || ahrs_p<-0.025 ? ahrs_p : 0))/*+g1*/)-kdrag*vy_theory)*dt/**0.667*/;
    
//    ax = -kdrag*vx_theory;
//    ay = kdrag*vy_theory;
    
    
//    vy_theory_ef =  vy_theory*cos_yaw + vx_theory*sin_yaw;
//    vx_theory_ef = -vy_theory*sin_yaw + vx_theory*cos_yaw;

//    float acc_b_x = Acc_f[1] ;
//    float acc_b_y = -Acc_f[0] ;
    
    float acc_b_x = ONE_G*sin_roll ;
    float acc_b_y = ONE_G*cos_roll*sin_pitch ;
    
//    applyDeadband(acc_b_x, 0.03);
//    applyDeadband(acc_b_y, 0.03);
    
    acc_ef.y = acc_b_x*sin_yaw + acc_b_y*cos_yaw;
    acc_ef.x = acc_b_x*cos_yaw - acc_b_y*sin_yaw;
    acc_ef.z = ONE_G*(accel_ef.z-1);
    
    vy_theory_ef += (/*ONE_G**/(acc_ef.y)-kdrag*vy_theory_ef)*dt;
    vx_theory_ef += (/*ONE_G**/(acc_ef.x)-kdrag*vx_theory_ef)*dt;
    
    
//    vy_fillered = GPS_vel_y_fillered*0.02 + (vy_theory_ef+vy_fillered)*0.98 ;
//    vx_fillered = GPS_vel_x_fillered*0.02 + (vx_theory_ef+vx_fillered)*0.98 ;
    
    //deleted gravity accel on axis
//    compute_dynamic_acceleration_vector(); //q to realworld



    //prepare to use q_conjugate
//  float qc0=q0;
//  float qc1=-q1;
//  float qc2=-q2;
//  float qc3=-q3;
//  
//  
//  
//    //q * q_compensate(0,g1,g2,g3)
//    float x1=q1;
//    float y1=q2;
//    float z1=q3;
//    float w1=q0;
//    //acc
//    float x2=g1;
//    float y2=g2;
//    float z2=g3;
//    float w2=0;
//            w = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2;
//            x = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2;
//            y = w1 * y2 + y1 * w2 + z1 * x2 - x1 * z2;
//            z = w1 * z2 + z1 * w2 + x1 * y2 - y1 * x2;
//    //(q*q_compensate) * q_c
//    x1=x;
//    y1=y;
//    z1=z;
//    w1=w;
//    
//    //compute_compensated_acc
//    x2=qc1;
//    y2=qc2;
//    z2=qc3;
//    w2=qc0;
//
//            w = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2;
//            x = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2;
//            y = w1 * y2 + y1 * w2 + z1 * x2 - x1 * z2;
//            z = w1 * z2 + z1 * w2 + x1 * y2 - y1 * x2;
    
     
    last_orig_altitude = alt;
    last_estimated_altitude = compute_altitude(dt, accel_ef.z-1, alt);   //moved to use earth-frame z accel
  
  }
}














//#define OUT_OF_BOUND 45                  //safe range for ultrasonic ,if it above this will much wrong
//#define anti_warp(x) (x>OUT_OF_BOUND ? 0 : 1)

//ชีดกลาง 1480 ขีดล่าง 1350

float control_to=0.0;
float desire_alt;
float hz_I=0;


#define lambda_alt 0.11
#define Kp_altitude 350   //350   change it when it slow to reach setpoint
#define Ki_altitude 100   //100   change it when it has offset 
#define Kd_altitude 200   //200   
#define Ka_altitude 150   //250   change it when it oscilation around true value





bool flag_altitude=false;
//void update_altitude();
//apply PI control
static void altitude_holding_on(float G_Dt){
  if(!flag_altitude)
  {
    //initialize altitude
    desire_alt = estimated_altitude;
    if(CH_THR > HOVERING_THROTTLE) HOVERING_THROTTLE_now=CH_THR;
    else HOVERING_THROTTLE_now = HOVERING_THROTTLE;
    flag_altitude = true;
  }
}

static void altitude_holding_off()
{
  flag_altitude=false;
  hz_I=0;
  flag_takeoff_processing=false;
}





//
//uint32_t  _last_good_update=0;
//float     _last_good_alt=0;
//float     _last_good_vel=0.0f;
//// initialise flags
//bool   baro_flags_initialised = 0;
//bool   baro_flags_glitching = 0;
//
//static void Check_alt()
//{
//  uint32_t now = millis();
//  float    sane_dt;                  // time since last sane baro reading
//  float    accel_based_distance=0;     // movement based on max acceleration
//  int32_t  alt_projected;            // altitude estimate projected from previous iteration
//  int32_t  distance_cm;              // distance from baro alt to current alt estimate in cm
//  bool     all_ok;                   // true if the new baro alt passes sanity checks
//  
//  
//  
//  // if not initialised or disabled update last good alt and exit
//    if (!baro_flags_initialised || !baro_glitch_enabled) {
//        _last_good_update = now;
//        _last_good_alt = (alt_ - alt_trim) *100.0f;
//        _last_good_vel = estimated_velocity *10.0f;
//        baro_flags_initialised = true;
//        baro_flags_glitching = false;
//        return;
//    }
//    // calculate time since last sane baro reading in ms
//    sane_dt = (now - _last_good_update) *0.001 ;
//    
//    
//    int32_t baro_alt = (alt_ - alt_trim) * 100.0f;
////    distance_cm = labs(_last_good_alt - baro_alt);
//    distance_cm = (_last_good_alt - baro_alt);
//    float a_proj=2*distance_cm*0.01/(sane_dt*sane_dt);
//    // estimate our alt from last known alt and velocity
////    alt_projected = _last_good_alt + (_last_good_vel * sane_dt);
////    alt_projected = _last_good_alt + 0.5*(ONE_G*(accel_ef.z-1)* sane_dt* sane_dt*100);
////
////    // calculate distance from recent baro alt to current estimate
////    int32_t baro_alt = (alt_ - alt_trim) * 100.0f;
////    // baro may have become unhealthy when calculating altitude
//////    if (!_baro_healthy) {
//////        baro_flags_glitching = true;
//////        return;
//////    }
////
////    // calculte distance from projected distance
////    distance_cm = labs(alt_projected - baro_alt);
////    // all ok if within a given hardcoded radius
////    if (distance_cm <= _dist_ok_cm) {
////        all_ok = true;
////    }else{
////        // or if within the maximum distance we could have moved based on our acceleration
////        
////        accel_based_distance = 0.5f * _accel_max_cmss * sane_dt * sane_dt;
////        
////        all_ok = (distance_cm <= accel_based_distance);
////    }
////    
//    float diff_a = labs(ONE_G*(accel_ef.z-1)-a_proj);
//    all_ok =1;
//    
//    saycomma(baro_alt);
//    saycomma(a_proj);
//    saycomma(ONE_G*(accel_ef.z-1));
//    sayend(diff_a);
//    
//    // store updates to baro position
//    if (all_ok) {
//        // position is acceptable
//        _last_good_update = now;
//        _last_good_alt = baro_alt;
//        _last_good_vel = estimated_velocity *10.0f;
//    }else {
//        alt_ = _last_good_alt*0.01;
//    }
//    
//    // update glitching flag
//    baro_flags_glitching= !all_ok;
//    
////    saycomma(accel_based_distance);
////    saycomma(alt_projected);
////    saycomma(baro_alt);
////    saycomma(distance_cm);
////    saycomma(estimated_altitude);
////    sayend(baro_flags_glitching);
//}

float vel_error[2]={0,0};
float vel_error_I[2]={0,0};
#define KI_vel 0.032  //0.05
#define LIMIT_VEL_BIAS 5000
bool init_vel_bias=false;

//for test
AP_BufferFloat_Size15   vel_f_hist_x;
AP_BufferFloat_Size15   vel_f_hist_y;

Vector2f v_hst_val;

//----------------------------------------------HORIZONTAL BIAS CALCELATION EXPERIMENTAL------------------------
static void debias_horizontal(float dt)
{

        // Initialization
            if (!init_vel_bias) {
              init_vel_bias = true;

            }
            
        // Estimation Error
              if (vel_f_hist_x.is_full()) {
                  v_hst_val.x = vel_f_hist_x.front();
              }else {
                  v_hst_val.x=0;
              }
              if (vel_f_hist_y.is_full()) {
                  v_hst_val.y = vel_f_hist_y.front();
              }else {
                  v_hst_val.y=0;
              }
         vel_error[0]   =  GPS_vel_x_fillered -  v_hst_val.x;
         vel_error[1]   =  GPS_vel_y_fillered -  v_hst_val.y;
         vel_error_I[0] =  vel_error_I[0] +  vel_error[0];
         vel_error_I[1] =  vel_error_I[1] +  vel_error[1];
         vel_error_I[0] =  constrain(vel_error_I[0],-LIMIT_VEL_BIAS,LIMIT_VEL_BIAS);  //changed from -2500 ,2500
         vel_error_I[1] =  constrain(vel_error_I[1],-LIMIT_VEL_BIAS,LIMIT_VEL_BIAS);  //changed from -2500 ,2500
         

        vx_fillered = vx_theory_ef*100 + vel_error_I[0]*KI_vel;
        vy_fillered = vy_theory_ef*100 + vel_error_I[1]*KI_vel;
        vy_theory = vy_fillered*cos_yaw-vx_fillered*sin_yaw;
        vy_theory*=0.01;
        vx_theory = vy_fillered*sin_yaw+vx_fillered*cos_yaw;
        vx_theory*=0.01;
        vel_f_hist_x.push_back(vx_fillered);
        vel_f_hist_y.push_back(vy_fillered);
//        vy_fillered = vy_theory_ef*100 + vel_error_I[1]*KI_vel;
//         inst_acceleration = compensated_acceleration * 9.80665 +  altitude_error_i *  KI;
//
//        // Integrators
//         delta =  inst_acceleration * dt + ( KP1 * dt) *  altitude_error;
//         estimated_altitude += ( estimated_velocity/5.0 +  delta) * (dt / 2) + ( KP2 * dt) *  altitude_error;
//         estimated_velocity +=  delta*10.0 ;
//         
//        return  estimated_altitude;
}
static void reset_vel() {
  vx_fillered = 0;
  vy_fillered = 0;
  vel_error[0]=0;
  vel_error[1]=0;
  vy_theory=0;
  vx_theory=0;
}
