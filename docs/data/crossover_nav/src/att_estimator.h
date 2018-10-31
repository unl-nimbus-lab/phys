#define att_FACTOR 1
#define aKP1  0.55*att_FACTOR // PI observer velocity gain 
#define aKP2  1.0*att_FACTOR  // PI observer position gain
#define aKI   0.001/att_FACTOR // PI observer integral gain (bias cancellation)
boolean att_initialized =false;
float   attitude_error_i[2] = {0,0};
float   attitude_error[2] = {0,0};
float   inst_acceleration_att[2] = {0.0,0.0};
float   _delta[2] = {0,0};
float   estimated_velocity_att[2] = {0.0,0.0};
float   estimated_attitude[2] = {0.0,0.0};

float   last_orig_attitude[2] = {0.0,0.0};
float   last_estimated_attitude[2] = {0.0,0.0};


static void compute_attitude(float dt,float compensated_acceleration_ef[2],float attitude[2])
{
//        if(GPS_fix==3) return;
        // Initialization
        if (!att_initialized) {
          att_initialized = true;
          estimated_attitude[0] = attitude[0];estimated_attitude[1] = attitude[1];
          estimated_velocity_att[0] = 0;      estimated_velocity_att[1] = 0;
          attitude_error[0]  = 0;             attitude_error[1]  = 0;
        }
            
        // Estimation Error  xy
         attitude_error[0]   =  attitude[0] -  estimated_attitude[0];
         attitude_error_i[0] =  attitude_error_i[0] +  attitude_error[0];
         attitude_error_i[0] =  constrain(attitude_error_i[0],-150,150);  //changed from -2500 ,2500
         attitude_error[1]   =  attitude[1] -  estimated_attitude[1];
         attitude_error_i[1] =  attitude_error_i[1] +  attitude_error[1];
         attitude_error_i[1] =  constrain(attitude_error_i[1],-150,150);  //changed from -2500 ,2500
         

         inst_acceleration_att[0] = compensated_acceleration_ef[0] * ONE_G +  attitude_error_i[0] *  aKI;
//          inst_acceleration_att[0] = acc_ef.x-kdrag*estimated_velocity_att[0]*0.1 +  attitude_error_i[0] *  aKI;
//          inst_acceleration_att[1] = acc_ef.y-kdrag*estimated_velocity_att[1]*0.1 +  attitude_error_i[1] *  aKI;
         inst_acceleration_att[1] = compensated_acceleration_ef[1] * ONE_G +  attitude_error_i[1] *  aKI;

        // Integrators
         _delta[0] =  inst_acceleration_att[0] * dt + ( aKP1 * dt) *  attitude_error[0];
         estimated_attitude[0] += ( estimated_velocity_att[0]/5.0 +  _delta[0]) * (dt / 2) + ( aKP2 * dt) *  attitude_error[0];
         estimated_velocity_att[0] +=  _delta[0]*10.0 ;
         
         _delta[1] =  inst_acceleration_att[1] * dt + ( aKP1 * dt) *  attitude_error[1];
         estimated_attitude[1] += ( estimated_velocity_att[1]/5.0 +  _delta[1]) * (dt / 2) + ( aKP2 * dt) *  attitude_error[1];
         estimated_velocity_att[1] +=  _delta[1]*10.0 ;
}

static void check_home2() {
    // get position move in lat, lon coordinates
    int32_t lat_offset = ahrs_home.lat - _last_home_lat;
    int32_t lng_offset = ahrs_home.lng - _last_home_lng;

    if (lat_offset != 0) {
        // calculate the position move in cm
//        float x_offset_cm = lat_offset * LATLON_TO_CM;
//
//        // move position
//        _position_base.x -= x_offset_cm;
//        _position.x -= x_offset_cm;
//
//        // update historic positions
//        for (uint8_t i = 0; i < _hist_position_estimate_x.size(); i++) {
//            float &x = _hist_position_estimate_x.peek_mutable(i);
//            x -= x_offset_cm;
//        }


        // update lon scaling
        _lon_to_cm_scaling = longitude_scale(ahrs_home) * LATLON_TO_CM;
    }

    if (lng_offset != 0) {
        // calculate the position move in cm
//        float y_offset_cm = lng_offset * _lon_to_cm_scaling;
//
//        // move position
//        _position_base.y -= y_offset_cm;
//        _position.y -= y_offset_cm;
//
//        // update historic positions
//        for (uint8_t i = 0; i < _hist_position_estimate_y.size(); i++) {
//            float &y = _hist_position_estimate_y.peek_mutable(i);
//            y -= y_offset_cm;
//        }
    }

    // store updated lat, lon position
    _last_home_lat = ahrs_home.lat;
    _last_home_lng = ahrs_home.lng;
}
static void convert_latlon_xy() {
   //_lon_to_cm_scaling = longitude_scale(ahrs_home) * LATLON_TO_CM;
    check_home2();
  // calculate distance from base location
    hy = ((float)(_GPS.lat - ahrs_home.lat) * LATLON_TO_CM)/**0.3+0.7*hy*/;
    hx = ((float)(_GPS.lng - ahrs_home.lng) * _lon_to_cm_scaling)/**0.3+0.7*hx*/;
}
