#include <Arduino.h>
#include "ekf.h"
#include "useful_fn.h"
#include <EEPROM.h>
#include <Wire.h>
#include "Ulink.h"
//#include "/home/fx/Dropbox/quad_utility/Ulink.h"
#include "rx_ppm.h"
#include "config.h"
//#include "simplo_rx.h"
#include "compass.h"
#include "MAF.h"
#include "gps.h"
#include "nav_test.h"
#include "mpu6050.h"
#include "ahrs.h"
//#include "sonar.h"
#include "alt.h"

#include "controlsystem.h"
#include "motor.h"
#include "att_estimator.h"

static void process_1000HZ();
static void process_100HZ();
static void process_50HZ();
static void process_33HZ();
static void process_15HZ();
static void process_10HZ();
static void pre_test_num_sensor();
static void test_num_readsensor();
uint8_t ADJUSTABLE_TASK();


uint32_t nub=0;


void setup()
{

  motor_command_all(1000);
  armed = 0;
  // initialize the serial link with processing
  begin_serial
#if defined(TEST_NUM_SENSOR)
#ifndef COMMUNICATE_VIA_WIRE
  Serial.begin(115200);
#endif
#endif

  Wire.begin();

  pinMode(RED_LED, OUTPUT); //LED A RED
  pinMode(YELLOW_LED, OUTPUT); //LED B YELLOW
  pinMode(GREEN_LED, OUTPUT); //LED C GREEN
  // I2C bus hardware specific settings
#if defined(__MK20DX128__)
  I2C0_F = 0x00; // 2.4 MHz (prescaler 20)
  I2C0_FLT = 4;
#endif
#if defined(__AVR__)
  TWBR = 12; // 400 KHz (maximum supported frequency)
#endif

  //-------RX INIT-------
  rx_initialize();
  delay(100);
  rx_update();
  delay(10);
  //-------PID LOAD-------
  calLoad_PID();
  //------IMU INIT--------
  mpu6050_initialize();
  MagHMC5883Int();
  //----Must read magneto first for no reason --> if not it will nan value
  compass_calibrate = AUX_2_STATUS();
  Mag_Calibrate();

  ahrs_initialize();
  delay(20);
  //------MOTOR INIT------
  motor_initialize();
  delay(20);

  baro.init(MS561101BA_ADDR_CSB_LOW);
  wait_for_baro();
  //  my3IMU.init(); // the parameter enable or disable fast mode
  mpu6050_Gyro_Calibrate();
  delay(200);


  setupFourthOrder();

  digitalWrite(RED_LED, LOW);

  previousTime = micros();


  GPS_INIT();


  //---------FOR TEST------------
  ahrs_home.lat =  _GPS.lat;
  ahrs_home.lng =  _GPS.lng;
//  INV_init();
  //    set_velocity_xy(0,0);
  setup_home_position();

}

void loop()
{
  // Timer
  currentTime = micros();

  // Read data (not faster then every 1 ms)

  if (currentTime - sensorPreviousTime >= 1000)   {
#if defined(MAF_MODE)
    MAF_GYRO();
    MAF_ACCEL();
#else
    mpu6050_readGyroSum();
    mpu6050_readAccelSum();
#endif
#if defined(TEST_NUM_SENSOR)
    pre_test_num_sensor();
#endif
  }
  //care gyro sample is must more than 4 sample before get calulation
  if (gyroSamples >= 4) {
    ////////////////////////////////////////100 HZ/////////////////////////////////////////////////
    if (currentTime - previousTime > 10000)
    {
      frameCounter++;
#if defined(TEST_NUM_SENSOR)
      test_num_readsensor();
#endif
      G_Dt = (currentTime - hundredHZpreviousTime) / 1000000.0;
      //        sayend(currentTime - hundredHZpreviousTime);
      hundredHZpreviousTime = currentTime;

#if defined(MAF_MODE)
      mpu6050_Get_gyro2();
      mpu6050_Get_accel2();
#else
      mpu6050_Get_gyro();
      mpu6050_Get_accel();
#endif

      //                Acc_f[0] = Acc_f[0] + (accel[XAXIS] - Acc_f[0])*0.121;//12.4  //Low pass filter ,smoothing factor  α := dt / (RC + dt)
      //                Acc_f[1] = Acc_f[1] + (accel[YAXIS] - Acc_f[1])*0.121;//12.4
      //                Acc_f[2] = Acc_f[2] + (accel[ZAXIS] - Acc_f[2])*0.121;//12.4
      // auxilary accel variable -> Acc_ft  is not use now
      Acc_f[0] = computeFourthOrder(accel[XAXIS], &fourthOrder[XAXIS]);
      Acc_f[1] = computeFourthOrder(accel[YAXIS], &fourthOrder[YAXIS]);
      Acc_f[2] = computeFourthOrder(accel[ZAXIS], &fourthOrder[ZAXIS]);


      ahrs_updateMARG(gyro[XAXIS], gyro[YAXIS], gyro[ZAXIS], Acc_f[0], Acc_f[1], Acc_f[2], c_magnetom_x, c_magnetom_y, c_magnetom_z, G_Dt);
      //trim it off
      basic_trim(ahrs_r, ahrs_p, ahrs_y);
      ahrs_r -= ahrs_r_trim;
      ahrs_p -= ahrs_p_trim;
      //                ahrs_y -= ahrs_y_trim;



      //<--------keep baro data -------->
      if (!flag_trim) {
        update(Acc_f[0], Acc_f[1], Acc_f[2], alt_ - alt_trim, G_Dt); //update baro acc fusion
        
        
        //Test Nav-----------------
        adaptive_filter(G_Dt) ;
//                          float attcom[2] ={hx*0.01,hy*0.01}; //convert m to cm
//                          float acccom[2] ={accel_ef.x,accel_ef.y};
//                          compute_attitude(G_Dt,acccom,attcom);

                  

      }



      //////////////////////////////////////////////////////////////////////////////////////////////
      ARM_DECISION();
      //////////////////////////////////////////////////////////////////////////////////////////////
      CONTROLSYSTEM();
      //////////////////////////////////////////////////////////////////////////////////////////////
      tomotor();
      //////////////////////////////////////////////////////////////////////////////////////////////

      ////////////////////////////////////////50 HZ/////////////////////////////////////////////////
      if (frameCounter % TASK_50HZ == 0) {
        G_Dt = (currentTime - fiftyHZpreviousTime) / 1000000.0;
        fiftyHZpreviousTime = currentTime;
        
        _barolast_update = millis();
        temperature = baro.getTemperature(MS561101BA_OSR_4096);
        press = baro.getPressure(MS561101BA_OSR_4096);
        pressure = baropress.process(press);
        alt_ = getAltitude;
        if (FLIGHT_MODE_CHECK == ALT_MODE || FLIGHT_MODE_CHECK == TAKE_OFF || FLIGHT_MODE_CHECK == VEL_HOLD)
        {
          // take off test
          altitude_holding_on(G_Dt);
        }/*else{ altitude_holding_off(); }*/

      }
      ////////////////////////////////////////33 HZ/////////////////////////////////////////////////
      if (frameCounter % TASK_33HZ == 0) {
        rx_update();
        Mag5883Read();

      }
      ////////////////////////////////////////25 HZ/////////////////////////////////////////////////
      if (frameCounter % TASK_25HZ == 0) {
        // for test
//        if (!flag_trim) update_NAV(0.04);

      }
      ////////////////////////////////////////15 HZ/////////////////////////////////////////////////
      if (frameCounter % TASK_15HZ == 0) {
        //////////////////CHECK IF NOT FLYING THEN CAN DOWNLOAD SOME USEFUL VALUE TO CHECK ON GROUND STATION
        if (!armed) {
          if (Serial3.available()) {
            RECIEVE_COMMAND_FROM_GROUND_STATION();
          }
        }
        /////////////////////////////////////////////////////////////////////////////////////////////////////
        FLIGHT_MODE(AUX_1);


        #if defined(USE_PROTOCAL)
//        /*if(att)*/ 
        imu_send(Acc_f[0],Acc_f[1],Acc_f[2],gyro[XAXIS],gyro[YAXIS],gyro[ZAXIS]);
        att_send(ahrs_r, ahrs_p, ahrs_y);
        alt_send(estimated_altitude, estimated_velocity);
//        cnt_send(control_roll, control_pitch, control_yaw);
//        radio_send(RF_ROLL, RF_PIT, CH_THR, RF_YAW, AUX_1, AUX_2);
        gps_send( (int) hy, (int) hx);
        barodata_send(alt_, temperature, press);
        nav_send( (int) Px_ef, (int) Py_ef);
//        motor_send(M1, M2, M3, M4);
//        float data[9] = {gyro[XAXIS],gyro[YAXIS],gyro[ZAXIS],g1,g2,vx_theory_ef,vy_theory_ef,ahrs_p,++nub};
//        flt_send(UNKNOWN_FLOAT,data,36);
        #endif  

        //test ekf
//        saytab(gyro[XAXIS]*100);
//        saytab(gyro[YAXIS]*100);
//        saytab(gyro[ZAXIS]*100);
//        saytab(9.80655*g1*100);
//        saytab(9.80655*g2*100);
//        saytab(ahrs_r*100);
//        sayend(ahrs_p*100);

//        saycomma(vx_theory_ef);
//        saycomma(vy_theory_ef);
        
        
        //long DT=micros();
        //Serial.print(ahrs_r);Serial.print(ahrs_p);Serial.print(ahrs_y);
        //Serial.print(estimated_altitude);Serial.print(estimated_velocity);
        //Serial.print(control_roll);Serial.print(control_roll);Serial.print(control_roll);
        //Serial.print(RF_ROLL);Serial.print(RF_ROLL);Serial.print(RF_ROLL);
        //Serial.print(RF_ROLL);Serial.print(RF_ROLL);Serial.println(RF_ROLL);



        //saycomma(estimated_altitude);
        //saycomma(desire_alt);
        ////saycomma(estimated_altitude);
        //saycomma(accel_ef.y);
        //sayend(accel_ef.z);

        //         saycomma(g1);
        //         saycomma(g2);
        //         saycomma(g3);
        //         saycomma(gyro[XAXIS]);
        //         saycomma(gyro[YAXIS]);
        //         saycomma(gyro[ZAXIS]);
        //         saycomma(estimated_altitude);
        //         sayend(altitude_error_i);


        //Serial.println(micros()-DT);


        //UPDATE MODE---------------------------------------------------------------------
        //                                saycomma(desire_alt - estimated_altitude);
        //                                saycomma(estimated_velocity);
        ////                                saycomma((g3 - 0.085)*Ka_altitude);
        //                                sayend(control_to);

        //saycomma(control_roll);
        //saycomma(control_pitch);
        //sayend(control_yaw);


        //---------calibrate alt hold-----------------------------------------------------
        //saycomma(desire_alt - estimated_altitude);
        //saycomma(desire_alt);
        ////saycomma(alt_-alt_trim);
        //saycomma(estimated_altitude);
        //saycomma(estimated_velocity);
        //saycomma(control_to);
        //sayend(control_yaw);
        ////Send_data();

        //if(gps) gps_send(millis(),millis()*2);

        //------------temporary------------------------------------------------------------

        //saycomma(desire_alt - estimated_altitude);
        //saycomma(vx_theory);
        //saycomma(M1);
        //saycomma(roll_desire);
        //say_mode();


        //------------RAW ACE
        //saytab(accel[XAXIS]);
        //saytab(accel[YAXIS]);
        //sayend(accel[ZAXIS]);
        //-------------check velocity-----------------------------------------------------
        //          saycomma(ahrs_r_trim);
        //          saycomma(ahrs_p_trim);
        //          saycomma(vx_theory);
        //          sayend(vy_theory);

        //-------------check velocity_control-----------------------------------------------
        //saycomma(roll_desire);
        //saycomma(vx_theory);





//        //saycomma(millis()/1000.0f);
//        saycomma(ahrs_r);
//        saycomma(ahrs_p);
//        //saycomma(g1);
//        //saycomma(g2);
//        saycomma(vx_theory);
//        sayend(vy_theory);






        //-------------check Roll pitch yaw ------------------------
        //saycomma(Acc_f[0]*ONE_G); //+
        //saycomma(Acc_f[1]*ONE_G);  //+
        //saycomma(Acc_f[2]*ONE_G);  //+
        //saycomma(Acc_ft[0]*ONE_G); //+
        //saycomma(Acc_ft[1]*ONE_G);  //+
        //sayend(Acc_ft[2]*ONE_G);  //+
        //
        //saycomma(c_magnetom_x);
        //saycomma(c_magnetom_y);
        //saycomma  (c_magnetom_z);
        //saytab(control_roll);
        //saytab(control_pitch);
        //saytab(control_yaw);
        //saycomma(gyro[XAXIS]); //+
        //saycomma(gyro[YAXIS]); //-
        //saycomma  (gyro[ZAXIS]); //-
        //saytab(estimated_altitude);
        //saytab(roll_desire);
        //saytab(pitch_desire);
        //saytab(AUX_1);
        //saytab(AUX_2);
        //saytab(AUX_3);
        //saytab(ahrs_r);
        //saytab(ahrs_p);
        //sayend(ahrs_y);
        //saycomma(CH_THR);

        //saycomma(g3);
        //sayend(control_to);

        //------------ MODE CHECK---------------------------------------------------------
        //saycomma(AUX_1);
        ////saycomma(FLIGHT_MODE_CHECK);
        //say_mode();

        //---------------test radio----------------------------------------------------
        //              saycomma(RF_ROLL);
        //              saycomma(RF_PIT);
        //              saycomma(RF_YAW);
        //              saycomma(CH_THR);
        //              saycomma(AUX_1);
        //              sayend(AUX_2);

        //---------------test velocity xy--------------------------------------------------
        //          saycomma(vx_theory);
        //          sayend(vx);


        //---------------test yaw + motor---------------------------------------------
        //            saycomma(control_yaw);
        //            saycomma(M1);
        //            saycomma(M2);
        //            saycomma(M3);
        //            sayend(M4);

        //---------test yaw
        //saycomma(err_yaw_level);
        //saycomma(err_yaw_rate);
        //saycomma(yaw_I_rate);
        //saycomma(yaw_D_rate);
        //sayend(control_yaw);

        //-------------test protect prop------------------------------------------

//                      saycomma(ahrs_r);
//                      saycomma(ahrs_p);
//                      sayend(ahrs_y);
        //              sayend(armed);


        //-----------test filter roll pitch-------------------------------------------
        //        saycomma(Acc_f[0]);
        //        saycomma(Acc_f[1]);
        //        saycomma(accel[YAXIS]);
        //        sayend(accel[XAXIS]);


        //------------test rpy and accel -----------------------------------
        //      saycomma(ahrs_r);
        //      saycomma(ahrs_p);
        //      sayend(Acc_f[2]);


        //        sayend(sonar_h);;
        //            saycomma(height);;
        //            saycomma(control_to);;
        //////            sayend(RF_PIT);;
        //            sayend(to_I);;

        //    saycomma(ax);
        //    saycomma(ay);
        //    saycomma(g1);
        //    sayend(g2);

        //            G_Dt = (currentTime - fifteenHZpreviousTime) / 1000000.0;

        //            Get_sonar();
        //fifteenHZpreviousTime = currentTime;
        //            magneto_lpf();
      }
      if (frameCounter % ADJUSTABLE_TASK() == 0)     {
        //FOR TEST

            G_Dt = (currentTime - lowPriorityTenHZpreviousTime) / 1000000.0;
            lowPriorityTenHZpreviousTime =  currentTime;
            
                            
                            correction_adaptive_filter(G_Dt);
                            convert_latlon_xy();
                            debias_horizontal(G_Dt);
                    
            //

//            saycomma(G_Dt);
//        Serial3.print(vx_theory_ef*100);Serial3.print(F(","));
//        Serial3.print(vy_theory_ef*100);Serial3.print(F(","));
//        Serial3.print(GPS_vel_x_fillered);Serial3.print(F(","));
//        Serial3.print(GPS_vel_y_fillered);Serial3.print(F(","));
            //        Serial3.print(vx_fillered);Serial3.print(F(","));
            ////                  Serial3.print(get_distance( _GPS.lat, _GPS.lng,lat_home,long_home));Serial3.print(F(","));


//            saycomma(estimated_attitude[0]);
//            saycomma(estimated_attitude[1]);
//            saycomma(vx


//            saycomma(vx_fillered);  //cm
//            saycomma(vy_fillered);
//            saycomma(GPS_vel_x_fillered);
//            saycomma(GPS_vel_y_fillered);
//           
//////            saycomma(_GPS.lat);
//////            saycomma(_GPS.lng);
//            saycomma(hx*0.01);
//            saycomma(hy*0.01);
//            saycomma(Dhx_b);
//            saycomma(Dhy_b);
//            sayend(GPS_hDOP);
//        Serial3.print(_GPS.lat,7);Serial3.print(F(","));
//        Serial3.print(_GPS.lng,7);Serial3.print(F(","));
//
//        Serial3.print(_position.x);Serial3.print(F(","));
//        Serial3.print(_position.y);Serial3.print(F(","));
////        Serial3.print(_position.z);Serial3.print(F(","));
//        Serial3.print(get_latitude());Serial3.print(F(","));
//        Serial3.print(get_longitude());Serial3.print(F(","));
//
//        Serial3.print(_velocity.x);Serial3.print(F(","));
//        Serial3.print(_velocity.y);Serial3.print(F(","));
//        Serial3.print(GPS_hDOP);Serial3.println(F(""));

      }

    // Reset frameCounter back to 0 after reaching 100 (1s)
    if (frameCounter >= 100) {

        frameCounter = 0;
      }

      previousTime = currentTime;
    }
  }

}





static void process_1000HZ() {
  //        sayend(currentTime - sensorPreviousTime);
  sensorPreviousTime = currentTime;
}
static void process_100HZ() {
}

static void process_50HZ() {
}
static void process_33HZ() {
  //            G_Dt = (currentTime - thitythreeHZpreviousTime) / 1000000.0;
  //            thitythreeHZpreviousTime = currentTime;
  //Get sonar_h variable for instant height in cm.
  // maf_pressure();  //try faster

}
static void process_15HZ() {

}


static void process_10HZ() {
//            G_Dt = (currentTime - lowPriorityTenHZpreviousTime) / 1000000.0;
//
//             currentTime = micros();
//
//             lowPriorityTenHZpreviousTime =  currentTime;
}
static void test_num_readsensor() {
  if (loopt % 50 == 0) {
//    Serial.println(loops);
    status_chk_send(loops);
    loops = 0;
    //          sayend(loops);
    //          sayend(s);
  }
  if (!flag_trim) loopt++;
}
static void pre_test_num_sensor() {
  if (!flag_trim) loops++;
}
uint8_t ADJUSTABLE_TASK() {
  return (10/*+G_Dt_sonar*4000*/);
}
void serialEvent1() {
  //statements
  while (Serial1.available()) {
    GPS_RECIEVE();
  }
}
