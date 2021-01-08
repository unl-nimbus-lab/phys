//#define Kp 0.2			// proportional gain governs rate of convergence to accelerometer/magnetometer
//#define Ki 0.0005		// integral gain governs rate of convergence of gyroscope biases
////#define Kp 6.0f			// proportional gain governs rate of convergence to accelerometer/magnetometer
////#define Ki 0.005f		// integral gain governs rate of convergence of gyroscope biases
//
//float exInt , eyInt , ezInt ;	// scaled integral error
////unsigned long last,now;
//
//float q0, q1 , q2 , q3 ;	// quaternion elements representing the estimated orientation
//float ahrs_p,ahrs_r,ahrs_y;






#include <math.h>


void ahrs_initialize()
{
  exInt=0;
  eyInt=0;
  ezInt=0;
  q0=1.0;
  q1=q2=q3=0;
}

static void ahrs_toEuler()
{
//  say(micros()-now1);sayend;
//  now1=micros();

  /* STANDARD ZYX
   y=atan2(2*q1*q2-2*q0*q3,2*q0*q0+2*q1*q1-1);
   p=-asin(2 * q1 * q3 + 2 * q0 * q2); // theta
   r=atan2(2 * q2 * q3 - 2 * q0 * q1, 2 * q0 * q0 + 2 * q3 * q3 - 1); // phi
   */
  ahrs_y=_atan2(2*q1*q2+2*q0*q3,2*q0*q0+2*q1*q1-1);
  ahrs_r=-_asin(2 * q1 * q3 - 2 * q0 * q2); // theta
  ahrs_p=_atan2(2 * q2 * q3 + 2 * q0 * q1, 2 * q0 * q0 + 2 * q3 * q3 - 1); // phi
}
static void Update_trig() {
//  sin_pitch = -DCM20;
  sin_pitch = sin(ahrs_p);
  cos_pitch = cos(ahrs_p);
  sin_roll = sin(ahrs_r);
  cos_roll = cos(ahrs_r);
//  cos_roll = cos_rollcos_pitch/cos_pitch;
  sin_yaw = sin(ahrs_y);
  cos_yaw = cos(ahrs_y);  
}

//------------------------------------------AHRS==========================================================================
static bool isSwitched(float previousError, float currentError) {
  if ((previousError > 0.0 &&  currentError < 0.0) || (previousError < 0.0 &&  currentError > 0.0)) {
    return true;
  }
  return false;
}
static void ahrs_updateMARG(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz, float G_Dt){

  float norm1,halfT;
  float hx, hy, hz, bx, bz;
  float vx, vy, vz, wx, wy, wz;
  float ex, ey, ez, ezMag;
  
  halfT=G_Dt/2.0;        
  // normalise the measurements
  norm1 = invSqrt(ax*ax + ay*ay + az*az);       
  ax = ax * norm1;
  ay = ay * norm1;
  az = az * norm1;
  norm1 = invSqrt(mx*mx + my*my + mz*mz);          
  mx = mx * norm1;
  my = my * norm1;
  mz = mz * norm1;         
  // compute reference direction of flux, Earth Frame
  hx = mx*DCM00 + my*DCM01 + mz*DCM02;
  hy = mx*DCM10 + my*DCM11 + mz*DCM12;
  hz = mx*DCM20 + my*DCM21 + mz*DCM22;         
  bx = 1/invSqrt((hx*hx) + (hy*hy));
  bz = hz; 
  // estimated direction of gravity and flux (v and w)
  vx = DCM20;//2*(q1q3 - q0q2)
  vy = DCM21;//2*(q0q1 + q2q3)
  vz = DCM22;//q0q0 - q1q1 - q2q2 + q3q3
  wx = bx*DCM00 + bz*DCM20;
  wy = bx*DCM01 + bz*DCM21;
  wz = bx*DCM02 + bz*DCM22;  
  // error is sum of cross product between reference direction of fields and direction measured by sensors
  ex = (ay*vz - az*vy);
  ey = (az*vx - ax*vz);
  ez = (ax*vy - ay*vx);
  //ez = (ax*vy - ay*vx);// + (mx*wy - my*wx)
  ezMag = (mx*wy - my*wx);
  // integral error scaled integral gain
  //exInt += ex*Ki*G_Dt;
  //eyInt += ey*Ki*G_Dt;
  //ezInt += (ez*Ki + ezMag*Ki_mag)*G_Dt;
  //*
  exInt += ex*Ki*G_Dt;
    if (isSwitched(previousEx,ex)) {
    exInt = 0.0;
  }
  previousEx = ex;
  eyInt += ey*Ki*G_Dt;
    if (isSwitched(previousEy,ey)) {
    eyInt = 0.0;
  }
  previousEy = ey;
  float ezi = (ez*Ki + ezMag*Ki_mag);
    ezInt += ezi*G_Dt;
    if (isSwitched(previousEz,ezi)) {
    ezInt = 0.0;
  }
  previousEz = ezi;
  //*/
  // adjusted gyroscope measurements
  gx += (Kp*ex + exInt);
  gy += (Kp*ey + eyInt);
  gz += (Kp*ez + ezMag*Kp_mag + ezInt);
  // integrate quaternion rate and normalise
  q0 = q0 + (-q1*gx - q2*gy - q3*gz)*halfT;
  q1 = q1 + (q0*gx + q2*gz - q3*gy)*halfT;
  q2 = q2 + (q0*gy - q1*gz + q3*gx)*halfT;
  q3 = q3 + (q0*gz + q1*gy - q2*gx)*halfT;  
  // normalise quaternion
  norm1 = invSqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
  q0 = q0 * norm1;
  q1 = q1 * norm1;
  q2 = q2 * norm1;
  q3 = q3 * norm1;
// auxiliary variables to reduce number of repeated operations
  //float q0q0 = q0*q0;
  float q0q1 = q0*q1;
  float q0q2 = q0*q2;
  float q0q3 = q0*q3;
  float q1q1 = q1*q1;
  float q1q2 = q1*q2;
  float q1q3 = q1*q3;
  float q2q2 = q2*q2;   
  float q2q3 = q2*q3;
  float q3q3 = q3*q3;  
  //direction cosine matrix (DCM), Rotation matrix , Rotated Frame to Stationary Frame XYZ
  //Quaternions_and_spatial_rotation
  DCM00 = 2*(0.5 - q2q2 - q3q3);//2*(0.5 - q2q2 - q3q3);//=q0q0 + q1q1 - q2q2 - q3q3
  DCM01 = 2*(q1q2 - q0q3);//2*(q0q1 + q2q3)
  DCM02 = 2*(q1q3 + q0q2);//2*(q1q3 - q0q2); 2*(q0q2 - q1q3)
  DCM10 = 2*(q1q2 + q0q3);
  DCM11 = 2*(0.5 - q1q1 - q3q3);//2*(0.5 - q1q1 - q3q3);//q0q0 - q1q1 + q2q2 - q3q3
  DCM12 = 2*(q2q3 - q0q1);
  DCM20 = 2*(q1q3 - q0q2);//-sin pitch
  DCM21 = 2*(q2q3 + q0q1);
  DCM22 = 2*(0.5 - q1q1 - q2q2);//2*(0.5 - q1q1 - q2q2);//=q0q0 - q1q1 - q2q2 + q3q3
  //DCM23 = q0*q0 - q1*q1 + q2*q2 - q3*q3;
  //ahrs_toEuler();
  ahrs_y=atan2(DCM10, DCM00);//2*q0*q0+2*q1*q1-1)
  ahrs_p=-asin(DCM20); // theta
  //ahrs_p=acos(22);//http://en.wikipedia.org/wiki/Rotation_formalisms_in_three_dimensions
  ahrs_r=atan2(DCM21, DCM22); // phi
  
    cos_rollcos_pitch = DCM22;
    if(cos_rollcos_pitch < 0.82)//25 deg = 0.82
    {
      cos_rollcos_pitch = 0.82;
    }
////transition between 0 and 360 or -180 and 180//////////    
//     Heading = ahrs_y - setHeading;
//    if (ahrs_y <= (setHeading - 180)) {
//      Heading += 360;
//    }
//    if (ahrs_y >= (setHeading + 180)) {
//      Heading -= 360;
//    } 
     ahrs_y = wrap_pi(ahrs_y + HALF_M_PI);
     
     
     
   
     
     
/////////////////////////////////////////////////////////
//////Earth Frame///////////////
//accrX_Earth = hx;
//accrY_Earth = hy;
//accrZ_Earth = hz;


//maybe acc_f[0] swarp and uncheck sign acc_f[1]

accel_ef.x = (Acc_f[0]*DCM00 + Acc_f[1]*DCM01 + Acc_f[2]*DCM02)*-1.0;
accel_ef.y = (Acc_f[0]*DCM10 + Acc_f[1]*DCM11 + Acc_f[2]*DCM12)*-1.0;
accel_ef.z = (Acc_f[0]*DCM20 + Acc_f[1]*DCM21 + Acc_f[2]*DCM22)/* - acc_offsetZ*/;

//accel_ef.normalized();
//norm(&accel_ef);



////accrZ_Earth = accrZ_Earth + (accrZ_Earthf - accrZ_Earth)*G_Dt*48.5;//18.5 Low pass filter ,smoothing factor  α := dt / (RC + dt)
applyDeadband(accel_ef.x, 0.01);//+-0.03 m/s^2
applyDeadband(accel_ef.y, 0.01);//+-0.03 m/s^2
//applyDeadband(accel_ef.z, 0.03);//+-0.03 m/s^2


Update_trig();
}





    // XY Axis specific variables
    bool                    _xy_enabled;                // xy position estimates enabled
    float                   _k1_xy;                     // gain for horizontal position correction
    float                   _k2_xy;                     // gain for horizontal velocity correction
    float                   _k3_xy;                     // gain for horizontal accelerometer offset correction
    uint32_t                _gps_last_update=0;           // system time of last gps update in ms
    uint32_t                _gps_last_time=0;             // time of last gps update according to the gps itself in ms
    uint8_t                 _historic_xy_counter=0;       // counter used to slow saving of position estimates for later comparison to gps
    AP_BufferFloat_Size5    _hist_position_estimate_x;  // buffer of historic accel based position to account for gpslag
    AP_BufferFloat_Size5    _hist_position_estimate_y;  // buffer of historic accel based position to account for gps lag
    float                   _lon_to_cm_scaling=LATLON_TO_CM;         // conversion of longitude to centimeters

    // Z Axis specific variables
    float                   _k1_z;                      // gain for vertical position correction
    float                   _k2_z;                      // gain for vertical velocity correction
    float                   _k3_z;                      // gain for vertical accelerometer offset correction
    uint32_t                _baro_last_update;          // time of last barometer update in ms
    AP_BufferFloat_Size15   _hist_position_estimate_z;  // buffer of historic accel based altitudes to account for barometer lag

    // general variables
    Vector3f                _position_base;             // (uncorrected) position estimate in cm - relative to the home location (_base_lat, _base_lon, 0)
    Vector3f                _position_correction;       // sum of corrections to _position_base from delayed 1st order samples in cm
    Vector3f                _velocity;                  // latest velocity estimate (integrated from accelerometer values) in cm/s
    Vector3f                _position_error;            // current position error in cm - is set by the check_* methods and used by update method to calculate the correction terms
    Vector3f                _position;                  // sum(_position_base, _position_correction) - corrected position estimate in cm - relative to the home location (_base_lat, _base_lon, 0)

    // error handling
//    GPS_Glitch&             _glitch_detector;           // GPS Glitch detector
//    Baro_Glitch&            _baro_glitch;               // Baro glitch detector
    uint8_t                 _error_count;               // number of missed GPS updates

    int32_t                 _last_home_lat;
    int32_t                 _last_home_lng;

static float longitude_scale(const Location &loc)
{
    static int32_t last_lat;
    static float scale = 1.0;
    if (labs(last_lat - loc.lat) < 100000) {
        // we are within 0.01 degrees (about 1km) of the
        // same latitude. We can avoid the cos() and return
        // the same scale factor.
        return scale;
    }
    scale = cosf(loc.lat * 1.0e-7f * DEG_TO_RAD);
    scale = constrain_float(scale, 0.01f, 1.0f);
    last_lat = loc.lat;
    return scale;
}

static void setup_home_position(void) 
{
    // set longitude to meters scaling to offset the shrinking longitude as we go towards the poles
    _lon_to_cm_scaling = longitude_scale(ahrs_home) * LATLON_TO_CM;

    // reset corrections to base position to zero
    // _position_base.x = 0.0f;
    // _position_base.y = 0.0f;
    // _position_correction.x = 0.0f;
    // _position_correction.y = 0.0f;
    // _position.x = 0.0f;
    // _position.y = 0.0f;
    _last_home_lat = ahrs_home.lat;
    _last_home_lng = ahrs_home.lng;

    // clear historic estimates
    // _hist_position_estimate_x.clear();
    // _hist_position_estimate_y.clear();

    // set xy as enabled
    // _xy_enabled = true;
}


// //-----------------------------------------3rd  order complementary filter by Arducopter (3.2)---------------------------------------------------------------------
// #define _time_constant_xy  2.5f // default time constant for complementary filter's X & Y axis
// #define _time_constant_z   5.0f // default time constant for complementary filter's Z axis


// // #defines to control how often historical accel based positions are saved
// // so they can later be compared to laggy gps readings
// #define AP_INTERTIALNAV_SAVE_POS_AFTER_ITERATIONS   3   //change from 10
// #define AP_INTERTIALNAV_GPS_LAG_IN_10HZ_INCREMENTS  4       // must not be larger than size of _hist_position_estimate_x and _hist_position_estimate_y
// #define AP_INTERTIALNAV_GPS_TIMEOUT_MS              300     // timeout after which position error from GPS will fall to zero

// #define AP_INERTIALNAV_LATLON_TO_CM 1.1113175


// static float longitude_scale(const Location &loc);
// static void set_altitude( float new_altitude);
//     /**
//      * correct_with_gps - calculates horizontal position error using gps
//      *
//      * @param now : current time since boot in milliseconds
//      * @param lon : longitude in 100 nano degrees (i.e. degree value multiplied by 10,000,000)
//      * @param lat : latitude  in 100 nano degrees (i.e. degree value multiplied by 10,000,000)
//      */
//     static void        correct_with_gps(uint32_t now, int32_t lon, int32_t lat);

//     /**
//      * check_home - checks if the home position has moved and offsets everything so it still lines up
//      */
//     static void check_home();

//     /**
//      * check_gps - checks if new gps readings have arrived and calls correct_with_gps to
//      * calculate the horizontal position error
//      * @see correct_with_gps(int32_t lon, int32_t lat, float dt);
//      */
//     static void        check_gps();

//     /**
//      * check_baro - checks if new baro readings have arrived and calls correct_with_baro to
//      * calculate the vertical position error
//      *
//      * @see correct_with_baro(float baro_alt, float dt);
//      */
//     static void        check_baro();

//     /**
//      * correct_with_baro - calculates vertical position error using barometer.
//      *
//      * @param baro_alt : altitude in cm
//      * @param dt : time since last baro reading in s
//      */
//     static void        correct_with_baro(float baro_alt, float dt);


//     /**
//      * update gains from time constant.
//      */
//     static void                    update_gains();

//     /**
//      * set_position_xy - overwrites the current position relative to the home location in cm
//      *
//      * the home location was set with AP_InertialNav::set_home_location(int32_t, int32_t)
//      *
//      * @param x : relative latitude  position in cm
//      * @param y : relative longitude position in cm
//      */
//     static void set_position_xy(float x, float y);

//     // XY Axis specific variables
//     bool                    _xy_enabled;                // xy position estimates enabled
//     float                   _k1_xy;                     // gain for horizontal position correction
//     float                   _k2_xy;                     // gain for horizontal velocity correction
//     float                   _k3_xy;                     // gain for horizontal accelerometer offset correction
//     uint32_t                _gps_last_update=0;           // system time of last gps update in ms
//     uint32_t                _gps_last_time=0;             // time of last gps update according to the gps itself in ms
//     uint8_t                 _historic_xy_counter=0;       // counter used to slow saving of position estimates for later comparison to gps
//     AP_BufferFloat_Size5    _hist_position_estimate_x;  // buffer of historic accel based position to account for gpslag
//     AP_BufferFloat_Size5    _hist_position_estimate_y;  // buffer of historic accel based position to account for gps lag
//     float                   _lon_to_cm_scaling=LATLON_TO_CM;         // conversion of longitude to centimeters

//     // Z Axis specific variables
//     float                   _k1_z;                      // gain for vertical position correction
//     float                   _k2_z;                      // gain for vertical velocity correction
//     float                   _k3_z;                      // gain for vertical accelerometer offset correction
//     uint32_t                _baro_last_update;          // time of last barometer update in ms
//     AP_BufferFloat_Size15   _hist_position_estimate_z;  // buffer of historic accel based altitudes to account for barometer lag

//     // general variables
//     Vector3f                _position_base;             // (uncorrected) position estimate in cm - relative to the home location (_base_lat, _base_lon, 0)
//     Vector3f                _position_correction;       // sum of corrections to _position_base from delayed 1st order samples in cm
//     Vector3f                _velocity;                  // latest velocity estimate (integrated from accelerometer values) in cm/s
//     Vector3f                _position_error;            // current position error in cm - is set by the check_* methods and used by update method to calculate the correction terms
//     Vector3f                _position;                  // sum(_position_base, _position_correction) - corrected position estimate in cm - relative to the home location (_base_lat, _base_lon, 0)

//     // error handling
// //    GPS_Glitch&             _glitch_detector;           // GPS Glitch detector
// //    Baro_Glitch&            _baro_glitch;               // Baro glitch detector
//     uint8_t                 _error_count;               // number of missed GPS updates

//     int32_t                 _last_home_lat;
//     int32_t                 _last_home_lng;
// //    static void check_home();
// //    static void set_position_xy(float x,float y);
    

// void INV_init() {
//   // X & Y axis time constant
//     if (_time_constant_xy == 0.0f) {
//         _k1_xy = _k2_xy = _k3_xy = 0.0f;
//     }else{
//         _k1_xy = 3.0f / _time_constant_xy;
//         _k2_xy = 3.0f / (_time_constant_xy*_time_constant_xy);
//         _k3_xy = 1.0f / (_time_constant_xy*_time_constant_xy*_time_constant_xy);
//     }

//     // Z axis time constant
//     if (_time_constant_z == 0.0f) {
//         _k1_z = _k2_z = _k3_z = 0.0f;
//     }else{
//         _k1_z = 3.0f / _time_constant_z;
//         _k2_z = 3.0f / (_time_constant_z*_time_constant_z);
//         _k3_z = 1.0f / (_time_constant_z*_time_constant_z*_time_constant_z);
//     }
// }






// // update - updates velocities and positions using latest info from ahrs and barometer if new data is available;
// static void update_NAV(float dt)
// {
//     // discard samples where dt is too large
//     if( dt > 0.1f ) {
//         return;
//     }

// //    // decrement ignore error count if required
// //    if (_flags.ignore_error > 0) {
// //        _flags.ignore_error--;
// //    }

//     // check if new baro readings have arrived and use them to correct vertical accelerometer offsets.
//     check_baro();

//     // check if home has moved and update
//     check_home();

//     // check if new gps readings have arrived and use them to correct position estimates
//     check_gps();
    
//     Vector3f accel_ef_ = accel_ef;   //compensated
//     accel_ef_.z -=1;
//     accel_ef_ *= (ONE_G*100.0f);
    
// //    Vector3f accel_ef_ = acc_ef;    //not compensated
// //    accel_ef_ *= (/*ONE_G**/100.0f);

//     // remove xy if not enabled
//     if( !_xy_enabled ) {
//         accel_ef_.x = 0.0f;
//         accel_ef_.y = 0.0f;
//     }

//     //Convert North-East-Down to North-East-Up
//     accel_ef_.z = -accel_ef_.z;

//     // convert ef position error to horizontal body frame
//     Vector2f position_error_hbf;
//     position_error_hbf.x = _position_error.x * cos_yaw + _position_error.y * sin_yaw;
//     position_error_hbf.y = -_position_error.x * sin_yaw + _position_error.y * cos_yaw;
    
    
//     float tmp = _k3_xy * dt;
//     accel_correction_hbf.x += position_error_hbf.x * tmp;
//     accel_correction_hbf.y += position_error_hbf.y * tmp;
//     accel_correction_hbf.z += _position_error.z * _k3_z  * dt;

//     tmp = _k2_xy * dt;
//     _velocity.x += _position_error.x * tmp;
//     _velocity.y += _position_error.y * tmp;
//     _velocity.z += _position_error.z * _k2_z  * dt;

//     tmp = _k1_xy * dt;
//     _position_correction.x += _position_error.x * tmp;
//     _position_correction.y += _position_error.y * tmp;
//     _position_correction.z += _position_error.z * _k1_z  * dt;

//     // convert horizontal body frame accel correction to earth frame
//     Vector2f accel_correction_ef;
//     accel_correction_ef.x = accel_correction_hbf.x * cos_yaw - accel_correction_hbf.y *  sin_yaw;
//     accel_correction_ef.y = accel_correction_hbf.x *  sin_yaw + accel_correction_hbf.y *  cos_yaw;


//     // calculate velocity increase adding new acceleration from accelerometers
//     Vector3f velocity_increase;
//     velocity_increase.x = (acc_ef.x-kdrag*_velocity.x)*dt;
//     velocity_increase.y = (acc_ef.y-kdrag*_velocity.y)*dt;

// //    velocity_increase.x = (accel_ef_.x + accel_correction_ef.x/* - _velocity.x*kdrag*/) * dt;  //implement with wind drag
// //    velocity_increase.y = (accel_ef_.y + accel_correction_ef.y/* - _velocity.y*kdrag*/) * dt;
// //    velocity_increase.z = (accel_ef_.z + accel_correction_hbf.z) * dt;

//     // calculate new estimate of position
//     _position_base += (_velocity + velocity_increase*0.5) * dt;

//     // update the corrected position estimate
//     _position = _position_base + _position_correction;

//     // calculate new velocity
// //    _velocity += velocity_increase;
//       _velocity.x = vx_fillered;
//       _velocity.y = vy_fillered;
// //      _velocity.z = estimated_velocity;

//     // store 3rd order estimate (i.e. estimated vertical position) for future use
//     _hist_position_estimate_z.push_back(_position_base.z);

//     // store 3rd order estimate (i.e. horizontal position) for future use at 10hz
//     _historic_xy_counter++;
//     if( _historic_xy_counter >= AP_INTERTIALNAV_SAVE_POS_AFTER_ITERATIONS ) {
//         _historic_xy_counter = 0;
//         _hist_position_estimate_x.push_back(_position_base.x);
//         _hist_position_estimate_y.push_back(_position_base.y);
//     }
// }
// //
// // XY Axis specific methods
// //

// // check_home - checks if the home position has moved and offsets everything so it still lines up
// static void check_home() {
//     if (!_xy_enabled) {
//         return;
//     }

//     // get position move in lat, lon coordinates
//     int32_t lat_offset = ahrs_home.lat - _last_home_lat;
//     int32_t lng_offset = ahrs_home.lng - _last_home_lng;

//     if (lat_offset != 0) {
//         // calculate the position move in cm
//         float x_offset_cm = lat_offset * LATLON_TO_CM;

//         // move position
//         _position_base.x -= x_offset_cm;
//         _position.x -= x_offset_cm;

//         // update historic positions
//         for (uint8_t i = 0; i < _hist_position_estimate_x.size(); i++) {
//             float &x = _hist_position_estimate_x.peek_mutable(i);
//             x -= x_offset_cm;
//         }


//         // update lon scaling
//         _lon_to_cm_scaling = longitude_scale(ahrs_home) * LATLON_TO_CM;
//     }

//     if (lng_offset != 0) {
//         // calculate the position move in cm
//         float y_offset_cm = lng_offset * _lon_to_cm_scaling;

//         // move position
//         _position_base.y -= y_offset_cm;
//         _position.y -= y_offset_cm;

//         // update historic positions
//         for (uint8_t i = 0; i < _hist_position_estimate_y.size(); i++) {
//             float &y = _hist_position_estimate_y.peek_mutable(i);
//             y -= y_offset_cm;
//         }
//     }

//     // store updated lat, lon position
//     _last_home_lat = ahrs_home.lat;
//     _last_home_lng = ahrs_home.lng;
// }
// // check_gps - check if new gps readings have arrived and use them to correct position estimates
// static void check_gps()
// {
//     const uint32_t now = millis();

//     // compare gps time to previous reading
//     const AP_GPS &gps = _GPS;

//     if(gps.last_fix_time_ms != _gps_last_time ) {

//         // call position correction method
//         correct_with_gps(now, gps.lng, gps.lat);

//         // record gps time and system time of this update
//         _gps_last_time = gps.last_fix_time_ms;
//     }else{
//         // if GPS updates stop arriving degrade position error to 10% over 2 seconds (assumes 100hz update rate)
//         if (now - _gps_last_update > AP_INTERTIALNAV_GPS_TIMEOUT_MS) {
//             _position_error.x *= 0.9886f;
//             _position_error.y *= 0.9886f;
// //            // increment error count
// //            if (_flags.ignore_error == 0 && _error_count < 255 && _xy_enabled) {
// //                _error_count++;
// //            }
//         }
//     }
// }
// // correct_with_gps - modifies accelerometer offsets using gps
// static void correct_with_gps(uint32_t now, int32_t lon, int32_t lat)
// {
//     float dt,x,y;
//     float hist_position_base_x, hist_position_base_y;

//     // calculate time since last gps reading
//     dt = (float)(now - _gps_last_update) * 0.001f;

//     // update last gps update time
//     _gps_last_update = now;

//     // discard samples where dt is too large
//     if( dt > 1.0f || dt == 0.0f || !_xy_enabled) {
//         return;
//     }

//     // calculate distance from base location
//     x = (float)(lat - ahrs_home.lat) * LATLON_TO_CM;
//     y = (float)(lon - ahrs_home.lng) * _lon_to_cm_scaling;

// //    // sanity check the gps position.  Relies on the main code calling GPS_Glitch::check_position() immediatley after a GPS update
// //    if (_glitch_detector.glitching()) {
// //        // failed sanity check so degrate position_error to 10% over 2 seconds (assumes 5hz update rate)
// //        _position_error.x *= 0.7943f;
// //        _position_error.y *= 0.7943f;
// //    }else{
// //        // if our internal glitching flag (from previous iteration) is true we have just recovered from a glitch
// //        // reset the inertial nav position and velocity to gps values
// //        if (_flags.gps_glitching) {
// //            set_position_xy(x,y);
// //            _position_error.x = 0.0f;
// //            _position_error.y = 0.0f;
// //        }else{
//             // ublox gps positions are delayed by 400ms
//             // we store historical position at 10hz so 4 iterations ago
//             if( _hist_position_estimate_x.is_full()) {
//                 hist_position_base_x = _hist_position_estimate_x.front();
//                 hist_position_base_y = _hist_position_estimate_y.front();
//             }else{
//                 hist_position_base_x = _position_base.x;
//                 hist_position_base_y = _position_base.y;
//             }

//             // calculate error in position from gps with our historical estimate
//             _position_error.x = x - (hist_position_base_x + _position_correction.x);
//             _position_error.y = y - (hist_position_base_y + _position_correction.y);
// //        }
// //    }

// //    // update our internal record of glitching flag so that we can notice a change
// //    _flags.gps_glitching = _glitch_detector.glitching();
// }


// // get accel based latitude
// static int32_t get_latitude()
// {
//     // make sure we've been initialised
//     if( !_xy_enabled ) {
//         return 0;
//     }

//     return ahrs_home.lat + (int32_t)(_position.x/LATLON_TO_CM);
// }

// // get accel based longitude
// static int32_t get_longitude()
// {
//     // make sure we've been initialised
//     if( !_xy_enabled ) {
//         return 0;
//     }
//     return ahrs_home.lng + (int32_t)(_position.y / _lon_to_cm_scaling);
// }

// static float longitude_scale(const Location &loc)
// {
//     static int32_t last_lat;
//     static float scale = 1.0;
//     if (labs(last_lat - loc.lat) < 100000) {
//         // we are within 0.01 degrees (about 1km) of the
//         // same latitude. We can avoid the cos() and return
//         // the same scale factor.
//         return scale;
//     }
//     scale = cosf(loc.lat * 1.0e-7f * DEG_TO_RAD);
//     scale = constrain_float(scale, 0.01f, 1.0f);
//     last_lat = loc.lat;
//     return scale;
// }
// // setup_home_position - reset state for home position change
// static void setup_home_position(void) 
// {
//     // set longitude to meters scaling to offset the shrinking longitude as we go towards the poles
//     _lon_to_cm_scaling = longitude_scale(ahrs_home) * LATLON_TO_CM;

//     // reset corrections to base position to zero
//     _position_base.x = 0.0f;
//     _position_base.y = 0.0f;
//     _position_correction.x = 0.0f;
//     _position_correction.y = 0.0f;
//     _position.x = 0.0f;
//     _position.y = 0.0f;
//     _last_home_lat = ahrs_home.lat;
//     _last_home_lng = ahrs_home.lng;

//     // clear historic estimates
//     _hist_position_estimate_x.clear();
//     _hist_position_estimate_y.clear();

//     // set xy as enabled
//     _xy_enabled = true;
// }

// //
// // Z Axis methods
// //

// // check_baro - check if new baro readings have arrived and use them to correct vertical accelerometer offsets
// static void check_baro()
// {
//     uint32_t baro_update_time;

//     // calculate time since last baro reading (in ms)
//     baro_update_time = _barolast_update;
//     if( baro_update_time != _baro_last_update ) {
//         const float dt = (float)(baro_update_time - _baro_last_update) * 0.001f; // in seconds
//         // call correction method
//         correct_with_baro(getAltitude*100.0f, dt);
//         _baro_last_update = baro_update_time;
//     }
// }

// // correct_with_baro - modifies accelerometer offsets using barometer.  dt is time since last baro reading
// static void correct_with_baro(float baro_alt, float dt)
// {
//     static uint8_t first_reads = 0;

//     // discard samples where dt is too large
//     if( dt > 0.5f ) {
//         return;
//     }

//     // discard first 10 reads but perform some initialisation
//     if( first_reads <= 10 ) {
//         set_altitude(baro_alt);
//         first_reads++;
//     }

// //    // sanity check the baro position.  Relies on the main code calling Baro_Glitch::check_alt() immediatley after a baro update
// //    if (_baro_glitch.glitching()) {
// //        // failed sanity check so degrate position_error to 10% over 2 seconds (assumes 10hz update rate)
// //        _position_error.z *= 0.89715f;
// //    }else{
// //        // if our internal baro glitching flag (from previous iteration) is true we have just recovered from a glitch
// //        // reset the inertial nav alt to baro alt
// //        if (_flags.baro_glitching) {
// //            set_altitude(baro_alt);
// //            _position_error.z = 0.0f;
// //        }else{
//             // 3rd order samples (i.e. position from baro) are delayed by 150ms (15 iterations at 100hz)
//             // so we should calculate error using historical estimates
//             float hist_position_base_z;
//             if (_hist_position_estimate_z.is_full()) {
//                 hist_position_base_z = _hist_position_estimate_z.front();
//             } else {
//                 hist_position_base_z = _position_base.z;
//             }

//             // calculate error in position from baro with our estimate
//             _position_error.z = baro_alt - (hist_position_base_z + _position_correction.z);
// //        }
// //    }
// //
// //    // update our internal record of glitching flag so that we can notice a change
// //    _flags.baro_glitching = _baro_glitch.glitching();
// }









// // get accel based latitude
// static float get_latitude_diff()
// {
//     // make sure we've been initialised
//     if( !_xy_enabled ) {
//         return 0;
//     }

//     return (_position.x/LATLON_TO_CM);
// }
// // get accel based longitude
// static float get_longitude_diff()
// {
//     // make sure we've been initialised
//     if( !_xy_enabled ) {
//         return 0.0f;
//     }

//     return (_position.y / _lon_to_cm_scaling);
// }
// // set_velocity_xy - set velocity in latitude & longitude directions (in cm/s)
// static void set_velocity_xy(float x, float y)
// {
//     _velocity.x = x;
//     _velocity.y = y;
// }
// // set_velocity_xy - set velocity in latitude & longitude directions (in cm/s)
// static float get_velocity_xy() 
// {
// 	return pythagorus(_velocity.x, _velocity.y);
// }
// // set_altitude - set base altitude estimate in cm
// static void set_altitude( float new_altitude)
// {
//     _position_base.z = new_altitude;
//     _position_correction.z = 0;
//     _position.z = new_altitude; // _position = _position_base + _position_correction
//     _hist_position_estimate_z.clear(); // reset z history to avoid fake z velocity at next baro calibration (next rearm)
// }

// // set_velocity_z - get latest climb rate (in cm/s)
// static void set_velocity_z(float z )
// {
//     _velocity.z = z;
// }
// // set_position_xy - sets inertial navigation position to given xy coordinates from home
// static void set_position_xy(float x, float y)
// {
//     // reset position from home
//     _position_base.x = x;
//     _position_base.y = y;
//     _position_correction.x = 0.0f;
//     _position_correction.y = 0.0f;

//     // clear historic estimates
//     _hist_position_estimate_x.clear();
//     _hist_position_estimate_y.clear();

//     // add new position for future use
//     _historic_xy_counter = 0;
//     _hist_position_estimate_x.push_back(_position_base.x);
//     _hist_position_estimate_y.push_back(_position_base.y);
// }

































struct fourthOrderData
{
  float  inputTm1,  inputTm2,  inputTm3,  inputTm4;
  float outputTm1, outputTm2, outputTm3, outputTm4;
} fourthOrder[4];

static float computeFourthOrder(float currentInput, struct fourthOrderData *filterParameters)
{
  // cheby2(4,60,12.5/50)
  #define _b0  0.001893594048567
  #define _b1 -0.002220262954039
  #define _b2  0.003389066536478
  #define _b3 -0.002220262954039
  #define _b4  0.001893594048567
  
  #define _a1 -3.362256889209355
  #define _a2  4.282608240117919
  #define _a3 -2.444765517272841
  #define _a4  0.527149895089809
  
  float output;
  
  output = _b0 * currentInput                + 
           _b1 * filterParameters->inputTm1  + 
           _b2 * filterParameters->inputTm2  +
           _b3 * filterParameters->inputTm3  +
           _b4 * filterParameters->inputTm4  -
           _a1 * filterParameters->outputTm1 -
           _a2 * filterParameters->outputTm2 -
           _a3 * filterParameters->outputTm3 -
           _a4 * filterParameters->outputTm4;

  filterParameters->inputTm4 = filterParameters->inputTm3;
  filterParameters->inputTm3 = filterParameters->inputTm2;
  filterParameters->inputTm2 = filterParameters->inputTm1;
  filterParameters->inputTm1 = currentInput;
  
  filterParameters->outputTm4 = filterParameters->outputTm3;
  filterParameters->outputTm3 = filterParameters->outputTm2;
  filterParameters->outputTm2 = filterParameters->outputTm1;
  filterParameters->outputTm1 = output;
    
  return output;
}

void setupFourthOrder()
{
  fourthOrder[XAXIS].inputTm1 = 0.0;
  fourthOrder[XAXIS].inputTm2 = 0.0;
  fourthOrder[XAXIS].inputTm3 = 0.0;
  fourthOrder[XAXIS].inputTm4 = 0.0;
  
  fourthOrder[XAXIS].outputTm1 = 0.0;
  fourthOrder[XAXIS].outputTm2 = 0.0;
  fourthOrder[XAXIS].outputTm3 = 0.0;
  fourthOrder[XAXIS].outputTm4 = 0.0;
  
  fourthOrder[YAXIS].inputTm1 = 0.0;
  fourthOrder[YAXIS].inputTm2 = 0.0;
  fourthOrder[YAXIS].inputTm3 = 0.0;
  fourthOrder[YAXIS].inputTm4 = 0.0;
  
  fourthOrder[YAXIS].outputTm1 = 0.0;
  fourthOrder[YAXIS].outputTm2 = 0.0;
  fourthOrder[YAXIS].outputTm3 = 0.0;
  fourthOrder[YAXIS].outputTm4 = 0.0;
  
  fourthOrder[ZAXIS].inputTm1 = 9.80655;
  fourthOrder[ZAXIS].inputTm2 = 9.80655;
  fourthOrder[ZAXIS].inputTm4 = 9.80655;
  
  fourthOrder[ZAXIS].outputTm1 = 9.80655;
  fourthOrder[ZAXIS].outputTm2 = 9.80655;
  fourthOrder[ZAXIS].outputTm3 = 9.80655;
  fourthOrder[ZAXIS].outputTm4 = 9.80655;
}

