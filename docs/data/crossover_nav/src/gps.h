/*
 * uBlox UBX Protocol Reader (runs on Arduino Leonardo, or equivalent)
 *
 * Note: RX pad on 3DR Module is output, TX is input
 */
MovingAvarageFilter vx(6);
MovingAvarageFilter vy(6);




#define MAX_LENGTH 512

#define  POSLLH_MSG  0x02
#define  SBAS_MSG    0x32
#define  VELNED_MSG  0x12
#define  STATUS_MSG  0x03
#define  SOL_MSG     0x06
#define  DOP_MSG     0x04
#define  DGPS_MSG    0x31

#define LONG(X)    *(long*)(&data[X])
#define ULONG(X)   *(unsigned long*)(&data[X])
#define INT(X)     *(int*)(&data[X])
#define UINT(X)    *(unsigned int*)(&data[X])


struct AP_GPS
{
     long lat;
     long lng;
     float alt;
     unsigned long last_fix_time_ms;
}_GPS;
Location ahrs_home;

unsigned char  state, lstate, code, id, chk1, chk2, ck1, ck2;
unsigned int  length, idx, cnt;

unsigned char data[MAX_LENGTH];


bool GPS_ALIVE = false;
bool _fix_ok = false;
long lastTime = 0;




 void sendCmd (unsigned char len, byte data[]) {
  Serial1.write(0xB5);
  Serial1.write(0x62);
  unsigned char chk1 = 0, chk2 = 0;
  for (unsigned char ii = 0; ii < len; ii++) {
    unsigned char cc = data[ii];
    Serial1.write(cc);
    chk1 += cc;
    chk2 += chk1;
  }
  Serial1.write(chk1);
  Serial1.write(chk2);
}

 void enableMsg (unsigned char id, boolean enable) {
  //               MSG   NAV   < length >  NAV
  byte cmdBuf[] = {0x06, 0x01, 0x03, 0x00, 0x01, id, enable ? 1 : 0};
  sendCmd(sizeof(cmdBuf), cmdBuf);
}

 void printHex (unsigned char val) {
//  if (val < 0x10)
    //Serial3.print("0");
  //Serial3.print(val, HEX);
}

 void GPS_INIT() {
  Serial1.begin(38400);
  if (false) {
    while(Serial3.available() == 0)
      ;
    lstate = state = 0;
  }
  // Modify these to control which messages are sent from module
  enableMsg(POSLLH_MSG, true);    // Enable position messages
  enableMsg(SBAS_MSG, true);      // Enable SBAS messages
  enableMsg(VELNED_MSG, true);    // Enable velocity messages
  enableMsg(STATUS_MSG, true);    // Enable status messages
  enableMsg(SOL_MSG, true);       // Enable soluton messages
  enableMsg(DOP_MSG, true);       // Enable DOP messages
  enableMsg(DGPS_MSG, false);     // Disable DGPS messages
}
static void GPS_GET_VALUE() {
  GPS_ALIVE = true;
  switch (id) {
    case 0x02:  // NAV-POSLLH
      ////Serial3.print(F("POSLLH: lon = "));
//       _GPS.lng = LONG(4);
//       _GPS.lat = LONG(8);
      _GPS.lat = LONG(8)/**0.000001*/;
      _GPS.lng = LONG(4)/**0.0000001*/;
      _GPS.alt = LONG(12);
                                                                      //      Serial3.print(GPS_vel_NORTH);         Serial3.print("\t"); 
                                                                      //      Serial3.print(GPS_vel_EAST);          Serial3.print("\t");  
                                                                      //      Serial3.print(GPS_vel_DOWN);          Serial3.print("\t"); 
                                                                      //      Serial3.print(GPS_Heading);           Serial3.print("\t"); 
                                                                      //      Serial3.print( _GPS.lng*0.0000001,8);  Serial3.print("\t"); 
                                                                      //      Serial3.print( _GPS.lat*0.0000001,8);   Serial3.print("\n"); 
//                  printLatLon(LONG(4));
//                  ////Serial3.print(F(", lat = "));
//                  printLatLon(LONG(8));
      ////Serial3.print(F(", vAcc = "));
//                  //Serial3.print(ULONG(24), DEC);
      ////Serial3.print(F(" mm, hAcc = "));
//                  //Serial3.print(ULONG(20), DEC);
      ////Serial3.print(F(" mm"));
     break;
    case 0x03:  // NAV-STATUS
      ////Serial3.print(F("STATUS: gpsFix = "));
      GPS_fix = data[4];
                                                              //      Serial3.print(GPS_fix);Serial3.print("\t");
                                                              //      Serial3.print(GPS_hDOP);Serial3.print("\t");
      //Serial3.print(data[4], DEC);
      if (data[5] & 2) {
        ////Serial3.print(F(", dgpsFix"));
      }
      break;
    case 0x04:  // NAV-DOP
      ////Serial3.print(F("DOP:    gDOP = "));
      //Serial3.print((float) UINT(4) / 100, 2);
      ////Serial3.print(F(", tDOP = "));
      //Serial3.print((float) UINT(8) / 100, 2);
      ////Serial3.print(F(", vDOP = "));
      //Serial3.print((float) UINT(10) / 100, 2);
      ////Serial3.print(F(", hDOP = "));
      GPS_hDOP = (float) UINT(12) * 0.01;
      //Serial3.print((float) UINT(12) / 100, 2);            ///////////////////////////////////////
      break;
    case 0x06:  // NAV-SOL
      ////Serial3.print(F("SOL:    week = "));
      //Serial3.print(UINT(8), DEC);
      ////Serial3.print(F(", gpsFix = "));                    ///////////////////////////////////////
      //Serial3.print(data[10], DEC);
      ////Serial3.print(F(", pDOP = "));
      //Serial3.print((float) UINT(44) / 100.0, 2);           ///////////////////////////////////////
      ////Serial3.print(F(", pAcc = "));
      //Serial3.print(ULONG(24), DEC);
      ////Serial3.print(F(" cm, numSV = "));
      _fix_ok = false;
      if(GPS_fix==3 /*3DFIX*/) {
      _fix_ok = true;
      _GPS.last_fix_time_ms = millis();
      }
      GPS_numSAT = data[47];
      //Serial3.print(data[47], DEC);                         ///////////////////////////////////////
      break;
    case 0x12:  // NAV-VELNED
      ////Serial3.print(F("VELNED: gSpeed = "));
      //Serial3.print(ULONG(20), DEC);                        ///////////////////////////////////////
      ////Serial3.print(F(" cm/sec, sAcc = "));
      //Serial3.print(ULONG(28), DEC);
      ////Serial3.print(F(" cm/sec, heading = "));

        GPS_vel_NORTH = ULONG(4);
        GPS_vel_EAST = ULONG(8);
        GPS_vel_DOWN = ULONG(12);
              GPS_vel_y_fillered = vy.process(GPS_vel_NORTH);
              GPS_vel_x_fillered = vx.process(GPS_vel_EAST);
        GPS_GroundSpeed = ULONG(20) ;
        GPS_Heading = (float) LONG(24) / 100000;

      //Serial3.print((float) LONG(24) / 100000, 2);          ///////////////////////////////////////
      ////Serial3.print(F(" deg, cAcc = "));
      //Serial3.print((float) LONG(32) / 100000, 2);
      ////Serial3.print(F(" deg"));
      break;
    default:
      break;
//    printHex(id);
  }
}
static void GPS_RECIEVE() {
  
  //RESET GPS STATUS
  GPS_ALIVE = false;
//                                                                                  Serial3.print("pass1");
  float dt=micros();
    unsigned char cc = Serial1.read();
    switch (state) {
      case 0:    // wait for sync 1 (0xB5)
        ck1 = ck2 = 0;
//                                                                                  Serial3.print("pass2");
        if (cc == 0xB5) {
          state++;
//                                                                                  Serial3.print("pass3");
        }
        break;
      case 1:    // wait for sync 2 (0x62)
        if (cc == 0x62)
          state++;
        else {
//          state = 0;                                                                Serial3.print("fail4");
        }
        break;
      case 2:    // wait for class code
        code = cc;
        ck1 += cc;
        ck2 += ck1;
        state++;
        break;
      case 3:    // wait for Id
        id = cc;
        ck1 += cc;
        ck2 += ck1;
        state++;
        break;
      case 4:    // wait for length byte 1
        length = cc;
        ck1 += cc;
        ck2 += ck1;
        state++;
        break;
      case 5:    // wait for length byte 2
        length |= (unsigned int) cc << 8;
        ck1 += cc;
        ck2 += ck1;
        idx = 0;
        state++;
        if (length > MAX_LENGTH)
          state= 0;
        break;
      case 6:    // wait for <length> payload bytes
        data[idx++] = cc;
        ck1 += cc;
        ck2 += ck1;
        if (idx >= length) {
          state++;
        }
        break;
      case 7:    // wait for checksum 1
        chk1 = cc;
        state++;
        break;
      case 8:    // wait for checksum 2
        chk2 = cc;
        boolean checkOk = ck1 == chk1  &&  ck2 == chk2;
        if (checkOk) {
          switch (code) {
            case 0x01:      // NAV-
              // Add blank line between time groups
              if (lastTime != ULONG(0)) {
                lastTime = ULONG(0);
                ////Serial3.print(F("\nTime: "));
//                Serial3.println(ULONG(0), DEC);
              }
//                                                                                  Serial3.print("pass4");
              GPS_GET_VALUE();
//                                                                                  Serial3.print("pass5");
//              Serial3.println();
              break;
            case 0x05:      // ACK-
              ////Serial3.print(F("ACK-"));
              switch (id) {
                case 0x00:  // ACK-NAK
                ////Serial3.print(F("NAK: "));
                break;
                case 0x01:  // ACK-ACK
                ////Serial3.print(F("ACK: "));
                break;
              }
//              printHex(data[0]);
              //Serial3.print(" ");
//              printHex(data[1]);
//              Serial3.println();
              break;
          }
        }
        state = 0;
        break;
    }    
  
 }






//  // Convert 1e-7 value packed into long into decimal format
//void printLatLon (long val) {
//  char buffer[14];
//  PString str(buffer, sizeof(buffer));
//  str.print(val, DEC);
//  char len = str.length();
//  char ii = 0;
//  while (ii < (len - 7)) {
//    Serial3.write(buffer[ii++]);
//  }
//  Serial3.write('.');
//  while (ii < len) {
//    Serial3.write(buffer[ii++]);
//  }
//}




