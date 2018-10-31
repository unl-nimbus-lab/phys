// #include "baro.h"
static void reset_vel();

//#define getAltitude ((pow((sea_press / press), 1/5.257) - 1.0) * (temperature + 273.15)) / 0.0065
#define getAltitude log(sea_press/pressure) * (temperature+273.15) * 29.271267f  //moving average fillered
#define getAltitude2 log(sea_press/press) * (temperature+273.15) * 29.271267f    //just use for startup


#include "calibration_edit.h"
    int16_t ACC_off_x, ACC_off_y, ACC_off_z, MAGN_off_x, MAGN_off_y, MAGN_off_z;
    float ACC_scale_x, ACC_scale_y, ACC_scale_z, MAGN_scale_x, MAGN_scale_y, MAGN_scale_z;


    MovingAvarageFilter baropress(15);
    const float sea_press = 1013.25;
    float press, temperature,pressure;
    float alt_;
    

//Altitude measure baro
// MS561101BA baro = MS561101BA();
// const float sea_press = 1013.25;
// float press, temperature,pressure;
// float alt_;

// static void maf_pressure() {
//   pressure = baropress.process(press);
// }

//float getAltitude(float press, float temp) {
//  //return (1.0f - pow(press/101325.0f, 0.190295f)) * 4433000.0f;
//  return ((pow((sea_press / press), 1/5.257) - 1.0) * (temp + 273.15)) / 0.0065;
//}
// static void wait_for_baro() {
//   do{
//     temperature = baro.getTemperature(MS561101BA_OSR_4096);
//     press = baro.getPressure(MS561101BA_OSR_4096);
//     Serial.print(".");
//     delay(100);
//   }while(press==0 || getAltitude2>1000);
//   sayend("");
//   sayend("Succesful reading barometer");
  
// //  press_last=baro.getPressure(MS561101BA_OSR_4096);
// }





// static void basic_trim(float ahrs_r,float ahrs_p,float ahrs_y) {
//   if(flag_trim){
//     count_for_trim++;
//       digitalWrite(RED_LED, HIGH);
//       digitalWrite(YELLOW_LED, HIGH);
//       delay(10);
//       digitalWrite(RED_LED, LOW);
//       digitalWrite(YELLOW_LED, LOW);
//       delay(10);
//     if(count_for_trim>=200)
//     { 
// //      ahrs_r_trim=ahrs_r;
// //      ahrs_p_trim=ahrs_p;
// ////      ahrs_y_trim=ahrs_y;
// //      RF_YAW_PLUS=ahrs_y;
      
//       temperature = baro.getTemperature(MS561101BA_OSR_4096);
//       press = baro.getPressure(MS561101BA_OSR_4096);
//       alt_trim=getAltitude;
// //      
// //
// //      
// //      g3_trim = g3;
      
      
      
//       //open some loop with flag_trim paremeter
//       flag_trim=false;
//     }
//         ahrs_home.lat =  _GPS.lat;
//         ahrs_home.lng =  _GPS.lng;
//         //for test
//         setup_home_position();
//   }

        
// }
static void arming_trim() {
        ahrs_r_trim=ahrs_r;
      ahrs_p_trim=ahrs_p;
//      ahrs_y_trim=ahrs_y;
      RF_YAW_PLUS=ahrs_y;
      
      
      // temperature = baro.getTemperature(MS561101BA_OSR_4096);
      // press = baro.getPressure(MS561101BA_OSR_4096);
      alt_trim=getAltitude;
      _barolast_update=millis();
      
      
      // reset_vel();
      
      g3_trim = g3;
      
      
      //GPS INITIAL
//      if(GPS_fix==3) {
        // ahrs_home.lat =  _GPS.lat;
        // ahrs_home.lng =  _GPS.lng;
        // //for test
        // set_velocity_xy(0,0);
        setup_home_position();
        
        // nav_setzero();
//      }
      
      
}
static void disarming_trim() {
      ahrs_r_trim=0;
      ahrs_p_trim=0;
      // temperature = baro.getTemperature(MS561101BA_OSR_4096);
      // press = baro.getPressure(MS561101BA_OSR_4096);
      alt_trim=getAltitude;
      _barolast_update=millis();
      
      // reset_vel();
      // //for test
      //   set_velocity_xy(0,0);
        setup_home_position();
}
