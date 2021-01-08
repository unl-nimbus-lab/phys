
//-----------------------------------------------------------------------------------------
// low-pass filter
// wl = alpha =  dt/(tau+dt)
// tau = 1/2pif = 1/omega
// omega = k(1-exp(-abs(Num_sat/(Num_sat-Num_sat_max))))  if Num_sat >3 and =0 elsewhere


float Px_ef,Py_ef;           // position earth-frame is OUTPUT
float alpha=0.3;             // adaptivable weight of complementary filter 
float alpha_m=1-alpha;       // h-pass weight
#define nsat_max 18          // max num_sat that gps can measure (estimate)
#define GPS_GroundSpeedmax  500 // max gps ground speed 
#define KP_CP 10
void adaptive_filter (float dt) {
  //on 100hz
  Px_ef+=vx_fillered*dt/*+0.5*acc_ef.x*dt*dt*/;
  Py_ef+=vy_fillered*dt/*+0.5*acc_ef.y*dt*dt*/;
}
void correction_adaptive_filter(float dt) {
  //on 1hz
  //GPS_GroundSpeed cm
  float omega_v = KP_CP*(1-fasterexp(-fabs(GPS_GroundSpeed/(GPS_GroundSpeed-GPS_GroundSpeedmax))));
  float omega_n = KP_CP*(1-fasterexp(-fabs(GPS_numSAT/(GPS_numSAT-nsat_max))));
  float omega = min(omega_v,omega_n);
  alpha = dt/(dt+(1/omega));
  alpha = constrain(alpha,0,1);
  alpha_m = 1-alpha;
  Serial.println(fasterexp(5));
  //on 10hz
  //-----------hp-----------lp-----
  Px_ef = Px_ef*alpha_m + alpha*hx;
  Py_ef = Py_ef*alpha_m + alpha*hy;
}
// from http://www.scielo.br/scielo.php?pid=S0104-65002009000300003&script=sci_arttext
// is also consider about low velocity will occure to precise of position provide from gps
void adaptive_parameter(float dt) {
  //on 1hz
  //GPS_GroundSpeed unscaled !!
  
  float omega_v = KP_CP*(1-fasterexp(-fabs(GPS_GroundSpeed/(GPS_GroundSpeed-GPS_GroundSpeedmax))));
  float omega_n = KP_CP*(1-fasterexp(-fabs(GPS_numSAT/(GPS_numSAT-nsat_max))));
  float omega = min(omega_v,omega_n);
  alpha = dt/(dt+(1/omega));
  alpha = constrain(alpha,0,1);
  alpha_m = 1-alpha;
}

void nav_setzero() {
  Px_ef = 0;
  Py_ef = 0;
}
