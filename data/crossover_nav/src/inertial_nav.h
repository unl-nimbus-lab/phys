float gps_eph = GPS_hAcc;  //m







 
//test gggg
//test git
//test git on sublime



float _w_xy_gps_v = 2.0f;
float _w_xy_gps_p = 1.0f;
float _w_acc_bias = 0.05f;
float _w_xy_res_v = 0.5f;

float _delay_gps  = 0.5f; //0.35  0.5f gps   0.1 msf
//msf test
//              _delay_gps  =0.1f;

float _w_xy_vision_p = 7.0f;
float _w_xy_vision_v = 0.0f;
//        bool cbrk_no_vision = true;


#define MIN_VALID_W 0.00001f
#define PUB_INTERVAL 10000  // limit publish rate to 100 Hz
#define EST_BUF_SIZE 250000 / PUB_INTERVAL //250000 / PUB_INTERVAL    // buffer size is 0.5s


int est_i = 0;
bool init_nav = false;
float est_buf[EST_BUF_SIZE][3][2];
bool ref_inited = false;
bool vision_valid = false;
//bool GPS_ALIVE = false;
float x_est_prev[2] = { 0.0f, 0.0f };
float y_est_prev[2] = { 0.0f, 0.0f };
float z_est_prev[2] = { 0.0f, 0.0f };

float baro_offset = 0.0f;   // baro offset for reference altitude, initialized on start, then adjusted



float x_est[2] = { 0.0f, 0.0f };  // pos, vel
float y_est[2] = { 0.0f, 0.0f };  // pos, vel
float z_est[2] = { 0.0f, 0.0f };  // pos, vel
float R_buf[EST_BUF_SIZE][3][3];  // rotation matrix buffer
float R_gps[3][3];      // rotation matrix for GPS correction moment




float acc[] = { 0.0f, 0.0f, 0.0f }; // N E D
//      float acc_bias[] = { 0.0f, 0.0f, 0.0f };  // body frame
float corr_baro = 0.0f;   // D
float corr_gps[3][2] = {
  { 0.0f, 0.0f },   // N (pos, vel)
  { 0.0f, 0.0f },   // E (pos, vel)
  { 0.0f, 0.0f },   // D (pos, vel)
};

float corr_vision[3][2] = {
  { 0.0f, 0.0f },   // N (pos, vel)
  { 0.0f, 0.0f },   // E (pos, vel)
  { 0.0f, 0.0f },   // D (pos, vel)
};


float w_gps_xy = 1.0f;
float w_gps_z = 1.0f;



static const float min_eph_epv = 2.0f;  // min EPH/EPV, used for weight calculation
static const float max_eph_epv = 20.0f; // max EPH/EPV acceptable for estimation

float eph = max_eph_epv;
float epv = 1.0f;

float eph_vision = 0.2f;
float epv_vision = 0.2f;

int buf_ptr = 0;
unsigned long t_prev;


float acc_bf[3] = {0.0f, 0.0f, 0.0f};

float vx_vision = 0, vy_vision = 0, vz_vision = 0;






//try
float w_gps_alt = 0.05;















void inertial_filter_predict(float dt, float x[2], float acc)
{
  if (isfinite(dt)) {
    if (!isfinite(acc)) {
      acc = 0.0f;
    }

    x[0] += x[1] * dt + acc * dt * dt / 2.0f;
    x[1] += acc * dt;
  }
}

void inertial_filter_correct(float e, float dt, float x[2], int i, float w)
{
  if (isfinite(e) && isfinite(w) && isfinite(dt)) {
    float ewdt = e * w * dt;
    x[i] += ewdt;

    if (i == 0) {
      x[1] += w * ewdt;
    }
  }
  // saytab(x_est[i]);sayend(y_est[i]);
}

static void vision_body_to_ef()
{

  vision_x[0] = -(vision_bx[0] * R[0][0] + vision_by[0] * R[0][1] + vision_bz[0] * R[0][2]);
  vision_y[0] = -(vision_bx[0] * R[1][0] + vision_by[0] * R[1][1] + vision_bz[0] * R[1][2]);
  vision_z[0] =  (vision_bx[0] * R[2][0] + vision_by[0] * R[2][1] + vision_bz[0] * R[2][2]);
  if (_fix_ok && ref_inited) {  // should re-check and add to desire value maintain old value (wo addition gps position to it base
    // can re-think about wait gps fix before getting to flight
    vision_x[0] += hx;
    vision_y[0] += hy;
  }
}

static void position_estimator() {

  ///////////////////////////////////////////////////////////////////////////////////////////////////
  unsigned long t = micros();
  gps_eph = GPS_hAcc;

  if (init_nav && !(vision_is_good || _fix_ok)) {
    init_nav = false;
    return;
  }
  if (!init_nav) {
    init_nav = true;
    ref_inited = true;

    x_est[0] = hx;
    x_est[1] = GPS_vel_EAST * 0.01;
    y_est[0] = hy;                   //m
    y_est[1] = GPS_vel_NORTH * 0.01; //m/s

    //          if(!_fix_ok && vision_is_good) {
    //
    //          }


    memset(est_buf, 0, sizeof(est_buf));
    memset(R_buf, 0, sizeof(R_buf));
    memset(R_gps, 0, sizeof(R_gps));

    memset(x_est_prev, 0, sizeof(x_est_prev));
    memset(y_est_prev, 0, sizeof(y_est_prev));
    //          memset(z_est_prev, 0, sizeof(z_est_prev));

    buf_ptr = 0;
  }


  if (vision_is_good) {
    vision_body_to_ef();
    static float last_vision_x = 0.0f;
    static float last_vision_y = 0.0f;
    static float last_vision_z = 0.0f;

    /* reset position estimate on first vision update */
    if (!vision_valid) {
      if (_fix_ok) {       //test when gps is fixed use them to reference
        x_est[0] = hx;
        x_est[1] = GPS_vel_EAST * 0.01;
        y_est[0] = hy;                   //m
        y_est[1] = GPS_vel_NORTH * 0.01; //m/s
      } else {
        x_est[0] = vision_x[0];
        x_est[1] = vision_x[1];
        y_est[0] = vision_y[0];
        y_est[1] = vision_y[1];
      }
      /* only reset the z estimate if the z weight parameter is not zero */
      //                        if (_w_z_vision_p > MIN_VALID_W)
      //                        {
      //                            z_est[0] = vision.z;
      //                            z_est[1] = vision.vz;
      //                        }

      vision_valid = true;

      last_vision_x = vision_x[0];
      last_vision_y = vision_y[0];
      last_vision_z = vision_z[0];

    }

    /* calculate correction for position */
    corr_vision[0][0] = vision_x[0] - x_est[0];
    corr_vision[1][0] = vision_y[0] - y_est[0];
    corr_vision[2][0] = vision_z[0] - z_est[0];


    float vision_dt = (vision_time_stamp - last_vision_time) / 1e3f;
    last_vision_time = vision_time_stamp;

    if (vision_dt > 0.000001f && vision_dt < 0.2f) {

      vision_x[1] = (vision_x[0] - last_vision_x) / vision_dt;
      vision_x[1] = vx_vision + 0.5 * (vision_x[1] - vx_vision);
      vx_vision   = vision_x[1];
      vision_y[1] = (vision_y[0] - last_vision_y) / vision_dt;
      vision_y[1] = vy_vision + 0.5 * (vision_y[1] - vy_vision);
      vy_vision   = vision_y[1];
      vision_z[1] = (vision_z[0] - last_vision_z) / vision_dt;
      vision_z[1] = vz_vision + 0.5 * (vision_z[1] - vz_vision);
      vz_vision   = vision_z[1];

      last_vision_x = vision_x[0];
      last_vision_y = vision_y[0];
      last_vision_z = vision_z[0];

      /* calculate correction for velocity */
      corr_vision[0][1] = vision_x[1] - x_est[1];
      corr_vision[1][1] = vision_y[1] - y_est[1];
      corr_vision[2][1] = vision_z[1] - z_est[1];
    } else {
      /* assume zero motion */
      corr_vision[0][1] = 0.0f - x_est[1];
      corr_vision[1][1] = 0.0f - y_est[1];
      corr_vision[2][1] = 0.0f - z_est[1];
    }

    //                    vision_updates++;
  }

  //                /* check for timeout on vision topic */
  if (vision_valid)
    if ((millis() > (vision_time_stamp + vision_topic_timeout))) {
      vision_valid = false;
    }
  /*try to renorm*/ // cant renorm because it cut acc that can exist any time
  // TODO make it more correct norm
  //        float norm1 = invSqrt(Acc_f[0]*Acc_f[0] + Acc_f[1]*Acc_f[1] + Acc_f[2]*Acc_f[2]);

  float Acc_fb[3];
  Acc_fb[0] = Acc_f[0];//*norm1;
  Acc_fb[1] = Acc_f[1];//*norm1;
  Acc_fb[2] = Acc_f[2];//*norm1;


  for (int i = 0; i < 3; i++) {
    Acc_fb[i] *= ONE_G;
    Acc_fb[i] -= acc_bias[i];
  }
  memcpy(acc_bf, Acc_fb, sizeof(Acc_fb));




  /* transform acceleration vector from body frame to NED frame */
  for (int i = 0; i < 3; i++) {
    acc[i] = 0.0f;
    for (int j = 0; j < 3; j++) {
      // if(i<2)
      //   acc[i] += (-R[i][j]) * acc_bf[j];  //invert
      // else
      acc[i] += R[i][j] * acc_bf[j];
    }
  }

  //        acc[2] -= ONE_G;

  //        corr_baro = baro_offset - getAltitude - z_est[0];



  /* calculate index of estimated values in buffer */
  int est_i = buf_ptr - 1 - Min(EST_BUF_SIZE - 1, Max(0, (int)(_delay_gps * 1000000.0f / PUB_INTERVAL)));
  if (est_i < 0) {
    est_i += EST_BUF_SIZE;
  }

  /* calculate correction for position */
  corr_gps[0][0] = hx - est_buf[est_i][0][0];
  corr_gps[1][0] = hy - est_buf[est_i][1][0];
  //    corr_gps[2][0] = alt_trim - gps_alt - est_buf[est_i][2][0];

  /* calculate correction for velocity */
  if (gps_vel_ned_valid) {
    corr_gps[0][1] = GPS_vel_EAST * 0.01 - est_buf[est_i][0][1];
    corr_gps[1][1] = GPS_vel_NORTH * 0.01 - est_buf[est_i][1][1];
    //    corr_gps[2][1] = gps.vel_d_m_s - est_buf[est_i][2][1];

  } else {
    corr_gps[0][1] = 0.0f;
    corr_gps[1][1] = 0.0f;
    corr_gps[2][1] = 0.0f;
  }
  /* save rotation matrix at this moment */
  memcpy(R_gps, R_buf[est_i], sizeof(R_gps));
  w_gps_xy = min_eph_epv / fmaxf(min_eph_epv, gps_eph);

  /* use GPS if it's valid and reference position initialized */


  float dt = t_prev > 0 ? (t - t_prev) / 1000000.0f : 0.0f;


  dt = fmaxf(fminf(0.02, dt), 0.002);   // constrain dt from 2 to 20 ms
  t_prev = t;

  /* increase EPH/EPV on each step */
  if (eph < max_eph_epv) {
    eph *= 1.0f + dt;
  }
  if (epv < max_eph_epv) {
    epv += 0.005f * dt; // add 1m to EPV each 200s (baro drift)
  }

  bool use_gps_xy = ref_inited && GPS_ALIVE && _w_xy_gps_p > MIN_VALID_W || (ref_inited );
  /* use VISION if it's valid and has a valid weight parameter */
  bool use_vision_xy = vision_valid && _w_xy_vision_p > MIN_VALID_W;
  //  bool use_vision_z = vision_valid && params.w_z_vision_p > MIN_VALID_W;

  bool can_estimate_xy = (eph < max_eph_epv) || use_gps_xy || use_vision_xy;

  float w_xy_gps_p = _w_xy_gps_p * w_gps_xy;
  float w_xy_gps_v = _w_xy_gps_v * w_gps_xy;

  float w_xy_vision_p = _w_xy_vision_p;
  float w_xy_vision_v = _w_xy_vision_v;
  //  float w_z_vision_p = _w_z_vision_p;

  /* accelerometer bias correction for GPS (use buffered rotation matrix) */
  float accel_bias_corr[3] = { 0.0f, 0.0f, 0.0f };
  if (use_gps_xy) {
    accel_bias_corr[0] -= corr_gps[0][0] * w_xy_gps_p * w_xy_gps_p;
    accel_bias_corr[0] -= corr_gps[0][1] * w_xy_gps_v;
    accel_bias_corr[1] -= corr_gps[1][0] * w_xy_gps_p * w_xy_gps_p;
    accel_bias_corr[1] -= corr_gps[1][1] * w_xy_gps_v;
  }
  /* transform error vector from NED frame to body frame */
  for (int i = 0; i < 3; i++) {
    float c = 0.0f;

    for (int j = 0; j < 3; j++) {
      //                      if(i<2)
      // c += (-R_gps[j][i]) * accel_bias_corr[j];
      //                      else
      c += R_gps[j][i] * accel_bias_corr[j];
    }

    if (isfinite(c)) {
      acc_bias[i] += c * _w_acc_bias * dt;
    }
  }
  
  /* accelerometer bias correction for VISION (use buffered rotation matrix) */
  accel_bias_corr[0] = 0.0f;
  accel_bias_corr[1] = 0.0f;
  accel_bias_corr[2] = 0.0f;

  // if (use_vision_xy) {
  //  accel_bias_corr[0] -= corr_vision[0][0] * w_xy_vision_p * w_xy_vision_p;
  //  accel_bias_corr[0] -= corr_vision[0][1] * w_xy_vision_v;
  //  accel_bias_corr[1] -= corr_vision[1][0] * w_xy_vision_p * w_xy_vision_p;
  //  accel_bias_corr[1] -= corr_vision[1][1] * w_xy_vision_v;
  //                 // transform error vector from NED frame to body frame
  //  for (int i = 0; i < 3; i++) {
  //    float c = 0.0f;

  //    for (int j = 0; j < 3; j++) {
  //                                if(i<2)
  //      c += (-R[j][i]) * accel_bias_corr[j];
  //                                else
  //                                c += (R[j][i]) * accel_bias_corr[j];
  //    }

  //    if (isfinite(c)) {
  //      acc_bias[i] += c * _w_acc_bias * dt;
  //    }
  //  }
  // }




  if (can_estimate_xy) {
    /* inertial filter prediction for position */
    inertial_filter_predict(dt, x_est, acc[0]);
    inertial_filter_predict(dt, y_est, acc[1]);

    if (!(isfinite(x_est[0]) && isfinite(x_est[1]) && isfinite(y_est[0]) && isfinite(y_est[1]))) {
      //      write_debug_log("BAD ESTIMATE AFTER PREDICTION", dt, x_est, y_est, z_est, x_est_prev, y_est_prev, z_est_prev, acc, corr_gps, w_xy_gps_p, w_xy_gps_v);
      memcpy(x_est, x_est_prev, sizeof(x_est));
      memcpy(y_est, y_est_prev, sizeof(y_est));
    }



    if (use_gps_xy) {
      eph = fminf(eph, gps_eph);


      inertial_filter_correct(corr_gps[0][0], dt, x_est, 0, w_xy_gps_p);
      inertial_filter_correct(corr_gps[1][0], dt, y_est, 0, w_xy_gps_p);

      if (gps_vel_ned_valid) {
        inertial_filter_correct(corr_gps[0][1], dt, x_est, 1, w_xy_gps_v);
        inertial_filter_correct(corr_gps[1][1], dt, y_est, 1, w_xy_gps_v);
      }
      // saytab(gps_vel_ned_valid);saytab(eph);saytab(can_estimate_xy);saytab(use_gps_xy);saytab(x_est[0]);sayend(y_est[0]);

    }
    //               if (use_vision_xy) {
    //  eph = fminf(eph, eph_vision);

    //  inertial_filter_correct(corr_vision[0][0], dt, x_est, 0, w_xy_vision_p);
    //  inertial_filter_correct(corr_vision[1][0], dt, y_est, 0, w_xy_vision_p);

    //  if (w_xy_vision_v > MIN_VALID_W) {
    //    inertial_filter_correct(corr_vision[0][1], dt, x_est, 1, w_xy_vision_v);
    //    inertial_filter_correct(corr_vision[1][1], dt, y_est, 1, w_xy_vision_v);
    //  }
    // }


    if (!(isfinite(x_est[0]) && isfinite(x_est[1]) && isfinite(y_est[0]) && isfinite(y_est[1]))) {
      //bad estimated
      memcpy(x_est, x_est_prev, sizeof(x_est));
      memcpy(y_est, y_est_prev, sizeof(y_est));
      memset(corr_gps, 0, sizeof(corr_gps));
      memset(corr_vision, 0, sizeof(corr_vision));
    } else {
      memcpy(x_est_prev, x_est, sizeof(x_est));
      memcpy(y_est_prev, y_est, sizeof(y_est));
    }
  } else {
    /* gradually reset xy velocity estimates */
    inertial_filter_correct(-x_est[1], dt, x_est, 1, _w_xy_res_v);
    inertial_filter_correct(-y_est[1], dt, y_est, 1, _w_xy_res_v);
  }


  /* push current estimate to buffer */
  est_buf[buf_ptr][0][0] = x_est[0];
  est_buf[buf_ptr][0][1] = x_est[1];
  est_buf[buf_ptr][1][0] = y_est[0];
  est_buf[buf_ptr][1][1] = y_est[1];
  //            est_buf[buf_ptr][2][0] = z_est[0];
  //            est_buf[buf_ptr][2][1] = z_est[1];

  /* push current rotation matrix to buffer */
  memcpy(R_buf[buf_ptr], R, sizeof(R));

  buf_ptr++;
  if (buf_ptr >= EST_BUF_SIZE) {
    buf_ptr = 0;
  }
  //        saytab(ahrs_r);saytab(ahrs_p);saytab(ahrs_y);

  //        saytab(GPS_vel_x_fillered);
  ////        saytab(hx);
  //        saytab(GPS_hAcc);
  //        saytab(GPS_numSAT);
  ////        sayend(GPS_hDOP);
  //       saytab(hx);saytab(hy);
  ////       saytab(GPS_vel_x_fillered);saytab(GPS_vel_y_fillered);
  //        saytab(x_est[0]);saytab(x_est[1]);
  //        saytab(y_est[0]);saytab(y_est[1]);
  ////        saytab(acc[0]);saytab(acc[1]);
  //        sayend(ahrs_y);;
  //        saytab(acc[0]);saytab(acc[1]);saytab(acc[2]);
  //        saytab(acc_bias[0]);saytab(acc_bias[1]);sayend(acc_bias[2]);




  //                float data[4] = {vision_x[0],vision_y[0],vision_x[1],vision_is_good};
  //                flt_send(UNKNOWN_FLOAT,data,4);
}







float FACTOR = 0.8;
float KP_V = 0.55*FACTOR; // PI observer velocity gain 
float KP_P = 1.0*FACTOR;  // PI observer position gain  (default = 1)
float KI_B = 0.001/FACTOR; // PI observer integral gain (bias cancellation)
bool initialized =false;
float   altitude_error_i_ = 0;
//float   acc_scale  = 0.0;
float   altitude_error = 0;
float   inst_acceleration_ = 0.0;
float   delta_ = 0;
float   estimated_velocity = 0.0;
float   estimated_altitude = 0.0;
 
float   last_orig_altitude = 0;
float   last_estimated_altitude = 0;
 
unsigned long t_prev_a;
 
inline static float compute_altitude(float dt, float compensated_acceleration,float altitude)
{



        // Initialization
            if (!initialized) {
              initialized = true;
              estimated_altitude = altitude ;
              // saytab("test init altitude");saytab(alt_-alt_trim);sayend(altitude);
              estimated_velocity = 0;
              altitude_error_i_ = 0;
            }
             
        // Estimation Error
         
         altitude_error   =  altitude -  estimated_altitude;
         altitude_error_i_ =  altitude_error_i_ +  altitude_error;
         altitude_error_i_ =  constrain(altitude_error_i_,-2500,2500);  //changed from -2500 ,2500
          
 
         inst_acceleration_ = compensated_acceleration * 9.80665 +  altitude_error_i_ *  KI_B;
 
        // Integrators
         delta_ =  inst_acceleration_ * dt + ( KP_V * dt) *  altitude_error;
 
          
         estimated_altitude += ( estimated_velocity/5.0 +  delta_) * (dt / 2) + ( KP_P * dt) *  altitude_error;
         estimated_velocity += delta_*10.0 ;
               //try to integrate with gps hz
         
          
        return  estimated_altitude;
}
 
 
 
 
inline static void update(float Acc_f[3],float alt) 
{

  unsigned long t = micros();

  float dt = t_prev_a > 0 ? (t - t_prev_a) / 1000000.0f : 0.0f;


  dt = fmaxf(fminf(0.02, dt), 0.002);   // constrain dt from 2 to 20 ms
  t_prev_a = t;


  if(!(alt!=alt)){
 
 
      float acc[3];

      /* transform acceleration vector from body frame to NED frame */
      for (int i = 0; i < 3; i++) {
        acc[i] = 0.0f;
        for (int j = 0; j < 3; j++) {
          // if(i<2)
          //   acc[i] += (-R[i][j]) * acc_bf[j];  //invert
          // else
          acc[i] += R[i][j] * Acc_f[j];
        }
      }



    last_orig_altitude = alt;
    last_estimated_altitude = compute_altitude(dt, acc[2]-1, alt);   //moved to use earth-frame z accel
   
  }
}
 
 
 