// #define HMC5883_Address 0x1E

int MagX,MagY,MagZ;
float MagXf,MagYf,MagZf;
float c_magnetom_x;
float c_magnetom_y;
float c_magnetom_z;
// static void LED_aux_green();
// static void LED_aux_red_warn();
// bool Save_compass_calibration_to_eeprom();
// bool Read_compass_calibration_from_eeprom();

// void MagHMC5883Int()
// {
//   Wire.beginTransmission(HMC5883_Address); //open communication with HMC5883
//   Wire.write(0x00); //Configuration Register A
//   Wire.write(0x70); //num samples: 8 ; output rate: 15Hz ; normal measurement mode
//   Wire.endTransmission();
//   delay(1);
//   Wire.beginTransmission(HMC5883_Address); //open communication with HMC5883
//   Wire.write(0x01); //Configuration Register B
//   Wire.write(0x20); //configuration gain 1.3Ga
//   Wire.endTransmission();
//   delay(1);
//   //Put the HMC5883 IC into the correct operating mode
//   Wire.beginTransmission(HMC5883_Address); //open communication with HMC5883
//   Wire.write(0x02); //select mode register
//   Wire.write(0x00); //continuous measurement mode
//   Wire.endTransmission();
//   delay(1);
// }

// void Mag5883Read()
// {
//   //Tell the HMC5883 where to begin reading data
//   Wire.beginTransmission(HMC5883_Address);
//   Wire.write(0x03); //select register 3, X MSB register
//   Wire.endTransmission();
//  //Read data from each axis, 2 registers per axis
//   Wire.requestFrom(HMC5883_Address, 6);
//  int i = 0;
//   byte result[6];
//   while(Wire.available())    
//   { 
//     result[i] = Wire.read(); 
//     i++;
//   }
//   Wire.endTransmission();   
//   MagX = ((result[0] << 8) | result[1]);//offset + 1.05
//   MagZ = ((result[2] << 8) | result[3])*-1;// + 0.05
//   MagY = ((result[4] << 8) | result[5])*-1;// - 0.55
//   MagXf = MagXf + (MagX - MagXf)*0.55;
//   MagYf = MagYf + (MagY - MagYf)*0.55;
//   MagZf = MagZf + (MagZ - MagZf)*0.55;
//  // adjust for  compass axis offsets/sensitivity differences by scaling to +/-5 range
//   c_magnetom_x = ((float)(MagXf - M_X_MIN) / (M_X_MAX - M_X_MIN))*10.0 - 5.0;
//   c_magnetom_y = (((float)(MagYf - M_Y_MIN) / (M_Y_MAX - M_Y_MIN))*10.0 - 5.0);
//   #ifndef DISABLE_INTERNAL_MAG
//     c_magnetom_z = -(((float)(MagZf - M_Z_MIN) / (M_Z_MAX - M_Z_MIN))*10.0 - 5.0);
//   #else
//     c_magnetom_z = (((float)(MagZf - M_Z_MIN) / (M_Z_MAX - M_Z_MIN))*10.0 - 5.0);
//   #endif
  
//   if(compass_calibrate && !armed && !AUX_2_STATUS())    //if disarm && calibration stick off && it was compass_calibration mode  all condition true will confirm 
//   {
//     compass_calibrate = false;                          //set for dont meet again in this condition
//     Save_compass_calibration_to_eeprom();
//   }
// }
// void Mag_Calibrate()//Calibration_sensor Magnetometer
// {
  
  
//   if(compass_calibrate) {
//     // Output MIN/MAX values
//     M_X_MIN = 0;
//     M_Y_MIN = 0;
//     M_Z_MIN = 0;
//     M_X_MAX = 0;
//     M_Y_MAX = 0;
//     M_Z_MAX = 0; 
//      sayend("Calibrating Compass plz turn around all axis");
//   }else{
//      sayend("USE OLD COMPASS CALIBRATION VALUE");
//      if(Read_compass_calibration_from_eeprom());
//   }
   

//     for (int i = 0; i < 600; i++) {//Calibration 30 s
//     if(compass_calibrate) {
//       digitalWrite(30, HIGH);
//       digitalWrite(13, HIGH);
//     }
//       Mag5883Read();
    
//       if(compass_calibrate) {
        
//         if (MagX < M_X_MIN) M_X_MIN = MagX;
//         if (MagX > M_X_MAX) M_X_MAX = MagX;
//         if (MagY < M_Y_MIN) M_Y_MIN = MagY;
//         if (MagY > M_Y_MAX) M_Y_MAX = MagY;
//         if (MagZ < M_Z_MIN) M_Z_MIN = MagZ;
//         if (MagZ > M_Z_MAX) M_Z_MAX = MagZ;
        
//         delay(25);
//         digitalWrite(RED_LED, LOW);
//         digitalWrite(GREEN_LED, LOW);
//         delay(25);
//       }
        
//     }
//       sayend("PUT ON GROUND");
//       LED_aux_green();   //it will wait 2 sec in function (see end this program)
//       //----wait 2 sec for put on ground
//       if(M_X_MIN<-1000 ||  M_Y_MIN<-1000 || M_Z_MIN<-1000 || M_X_MAX>1000 || M_Y_MAX>1000 || M_Z_MAX>1000) {
//           sayend("----failed compass calibration--go--use--default--value----"); 
//           M_X_MIN = -493;
//           M_X_MAX = 421; 
//           M_Y_MIN = -356;
//           M_Y_MAX = 568; 
//           M_Z_MIN = -396;
//           M_Z_MAX = 399; 
//           LED_aux_red_warn();
//         }
      
//       say("magn x,y,z (min/max) = ");
//       delay(20);
//       saycomma(M_X_MIN);
//       delay(20);
//       saytab(M_X_MAX);delay(20);
//       saycomma(M_Y_MIN);delay(20);
//       saytab(M_Y_MAX);delay(20);
//       saycomma(M_Z_MIN);delay(20);
//       sayend(M_Z_MAX);

// //      delay(20);
// //      say("SCALE_x=");sayend((M_X_MAX-M_X_MIN)/2); 
// //      delay(20);
// //      say("SCALE_y=");sayend((M_Y_MAX-M_Y_MIN)/2); 
// //      delay(20);
// //      say("SCALE_z=");sayend((M_Z_MAX-M_Z_MIN)/2); 
// //      delay(20);
// //      say("offset_x=");sayend((M_X_MAX+M_X_MIN)/2); 
// //      delay(20);
// //      say("offset_y=");sayend((M_Y_MAX+M_Y_MIN)/2); 
// //      delay(20);
// //      say("offset_z=");sayend((M_Z_MAX+M_Z_MIN)/2); 
      
      
// }

// static void LED_aux_green() {
//     for (int i =0; i<10 ; i++ ) {
//         digitalWrite(GREEN_LED, HIGH);
//         delay(250);
//         digitalWrite(GREEN_LED, LOW);
//         delay(250);
//     }
// }
// static void LED_aux_red_warn() {
//     for (int i =0; i<10 ; i++ ) {
//         digitalWrite(RED_LED, HIGH);
//         delay(250);
//         digitalWrite(RED_LED, LOW);
//         delay(50);
//     }
// }
// bool Save_compass_calibration_to_eeprom() {
//   int16_t data[6] = { M_X_MIN,M_X_MAX,M_Y_MIN,M_Y_MAX,M_Z_MIN,M_Z_MAX };  
  
//   const uint8_t eepromsize = sizeof(int16_t) * 6 /*+ sizeof(int) * 6*/;
//   uint8_t j = 0;
//   EEPROM.write(TD_EEPROM_BASE, TD_EEPROM_SIGNATURE);
  
//   for(uint8_t i = 1; i<(eepromsize)+1; i++) {
//     if(i%2!=0) {
//       EEPROM.write(TD_EEPROM_BASE + 24 + i,uint8_t((data[j]<<8)>>8) );   //send 1 byte
//       saycomma(i);
//     }else{
//       EEPROM.write(TD_EEPROM_BASE + 24 + i,uint8_t(data[j]>>8) );      //send another byte
//       saycomma(i);
//       j++;
//     }
    
//   }
//   return true;
// }
// bool Read_compass_calibration_from_eeprom() {
//   if(EEPROM.read(TD_EEPROM_BASE) == TD_EEPROM_SIGNATURE) { // check if signature is ok so we have good data
//   location = TD_EEPROM_BASE + 25; // reset location  (+0 =Signature   +1-+24  PPID  +25-+36  COMPASS

//   //COMPASS---------------------------------------------
//     eeprom_read_var(sizeof(M_X_MIN), (byte *) &M_X_MIN);
//     eeprom_read_var(sizeof(M_X_MAX), (byte *) &M_X_MAX);
//     eeprom_read_var(sizeof(M_Y_MIN), (byte *) &M_Y_MIN);
//     eeprom_read_var(sizeof(M_Y_MAX), (byte *) &M_Y_MAX);
//     eeprom_read_var(sizeof(M_Z_MIN), (byte *) &M_Z_MIN);
//     eeprom_read_var(sizeof(M_Z_MAX), (byte *) &M_Z_MAX);
    
//   return true;
//   }
// }
