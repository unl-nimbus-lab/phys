#define HEADER 0xb5

//#define Sayout(x) Serial.write(x);
#define toLong(X)    *(long*)(&X)
#define sendbyte(n)    Serial3.write(uint8_t(n))
#define toUint(X)    *(uint16_t*)(&X)
#define toSint(X)    *(int16_t*)(&X)
#define toFlt(X)    *(float*)(&X)


//SoftwareSerial3 mySerial3(10, 11); // RX, TX



//data Streams
// #define DATA_ATT        1
// #define DATA_ALL_RAW    2
// #define DATA_RAW_IMU    3
// #define DATA_RAW_BARO   4
// #define DATA_RAW_RADIO  5
// #define DATA_GPS        6
// #define DATA_MOTOR      7
// #define DATA_CNT_VAL    8
// #define DATA_STATUS     9
// #define DATA_ALLTITUDE  10
// #define DATA_V_VAL      11
// #define DATA_ACC_AXIS   12

//msg_id
#define ATTITUDE        1 //float 3
#define RAW_IMU         2 //float 6
#define RAW_BARO        3 //float 3
#define RAW_RADIO       4 //int 5
#define MOTOR_OUT       5 //int 4
#define ALTITUDE        6 //float 3
#define ACC_AXIS        7 //float 3
#define V_VAL           8 //float 2
#define CNT_VAL         9 //float 3 cnt roll,pitch,yaw
#define STATUS_CHK      10 //int 1
#define GPS             11 //long 2
#define DATA_REQUEST    12 //chr 1
#define CMD_TAKEOFF     13 //chr 1
#define UNKNOWN_TYPE     14 //unknown
#define UNKNOWN_FLOAT    15
#define CONT_COMPASS     16
#define CONT_PID         17
#define NAV_DATA         18

bool alive = false;
bool att = false;
bool gps = false;


long time,last_;

//---------------------Send Data Medthod-------------------
static void send4byte(long n) {
    Serial3.write(uint8_t((n<<24)>>24));
    Serial3.write(uint8_t((n<<16)>>24));
    Serial3.write(uint8_t((n<<8)>>24));
    Serial3.write(uint8_t(n>>24));
    
}

static void send2byte(int n) {
    Serial3.write(uint8_t((n<<8)>>8));
    Serial3.write(uint8_t(n>>8));

}
static uint16_t sum4byte(long n) {
  return uint8_t(n>>24) + uint8_t((n<<8)>>24) + uint8_t((n<<16)>>24)+uint8_t((n<<24)>>24);
}
static uint16_t sum2byte(uint16_t n) {
  return uint8_t(n>>8) + uint8_t((n<<8)>>8);
}
// static uint16_t sum2byte(uint16_t n) {
//   return uint8_t(n>>16) + uint8_t((n<<8)>>16) ;
// }
//------------------------------------------------------------
static void att_send(float _n1,float _n2,float _n3) {
  
    long n1=toLong(_n1);
    long n2=toLong(_n2);
    long n3=toLong(_n3);
    uint16_t chk;
    uint8_t len;
    len =12;
    chk = HEADER + len + ATTITUDE +sum4byte(n1)+sum4byte(n2)+sum4byte(n3);
    //Serial3.print("ATT");
   // Serial3.println(n1);
    sendbyte(HEADER); //hdr
    sendbyte(len);    //len
    sendbyte(ATTITUDE); //msg id
    send4byte(n1); //data1
    send4byte(n2); //data2
    send4byte(n3); //data3
    send2byte(chk); //chk
}
static void imu_send(float _n1,float _n2,float _n3,float _n4,float _n5,float _n6) {
  
    long n1=toLong(_n1);
    long n2=toLong(_n2);
    long n3=toLong(_n3);
    long n4=toLong(_n4);
    long n5=toLong(_n5);
    long n6=toLong(_n6);
    uint16_t chk;
    uint8_t len;
    len =24;
    chk = HEADER + len + RAW_IMU +sum4byte(n1)+sum4byte(n2)+sum4byte(n3)+sum4byte(n4)+sum4byte(n5)+sum4byte(n6);
    //Serial3.print("ATT");
   // Serial3.println(n1);
    sendbyte(HEADER); //hdr
    sendbyte(len);    //len
    sendbyte(RAW_IMU); //msg id
    send4byte(n1); //data1
    send4byte(n2); //data2
    send4byte(n3); //data3
    send4byte(n4); //data1
    send4byte(n5); //data2
    send4byte(n6); //data3
    send2byte(chk); //chk
}
static void cnt_send(float _n1,float _n2,float _n3) {
  
    long n1=toLong(_n1);
    long n2=toLong(_n2);
    long n3=toLong(_n3);
    uint16_t chk;
    uint8_t len;
    len =12;
    chk = HEADER + len + CNT_VAL +sum4byte(n1)+sum4byte(n2)+sum4byte(n3);
    //Serial3.print("ATT");
   // Serial3.println(n1);
    sendbyte(HEADER); //hdr
    sendbyte(len);    //len
    sendbyte(CNT_VAL); //msg id
    send4byte(n1); //data1
    send4byte(n2); //data2
    send4byte(n3); //data3
    send2byte(chk); //chk
}
static void radio_send(int _n1,int _n2,int _n3,int _n4,int _n5,int _n6) {
  
    int n1=toUint(_n1);
    int n2=toUint(_n2);
    int n3=toUint(_n3);
    int n4=toUint(_n4);
    int n5=toUint(_n5);
    int n6=toUint(_n6);
    uint16_t chk;
    uint8_t len;
    len =12;
    chk = HEADER + len + RAW_RADIO +sum2byte(n1)+sum2byte(n2)+sum2byte(n3)+sum2byte(n4)+sum2byte(n5)+sum2byte(n6);
    //Serial3.print("ATT");
   // Serial3.println(n1);
    sendbyte(HEADER); //hdr
    sendbyte(len);    //len
    sendbyte(RAW_RADIO); //msg id
    send2byte(n1); //data1
    send2byte(n2); //data2
    send2byte(n3); //data3
    send2byte(n4); //data1
    send2byte(n5); //data2
    send2byte(n6); //data3
    send2byte(chk); //chk
}
static void motor_send(int n1,int n2,int n3,int n4) {
  
    uint16_t chk;
    uint8_t len;
    len =8;
    chk = HEADER + len + MOTOR_OUT +sum2byte(n1)+sum2byte(n2)+sum2byte(n3)+sum2byte(n4);
    //Serial3.print("ATT");
   // Serial3.println(n1);
    sendbyte(HEADER); //hdr
    sendbyte(len);    //len
    sendbyte(MOTOR_OUT); //msg id
    send2byte(n1); //data1
    send2byte(n2); //data2
    send2byte(n3); //data3
    send2byte(n4); //data1
    send2byte(chk); //chk
}
static void status_chk_send(int n1) {
  
    uint16_t chk;
    uint8_t len;
    len =2;
    chk = HEADER + len + STATUS_CHK +sum2byte(n1);
    //Serial3.print("ATT");
   // Serial3.println(n1);
    sendbyte(HEADER); //hdr
    sendbyte(len);    //len
    sendbyte(STATUS_CHK); //msg id
    send2byte(n1); //data1
    send2byte(chk); //chk
}

static void barodata_send(float _n1,float _n2,float _n3) {
  
    long n1=toLong(_n1);
    long n2=toLong(_n2);
    long n3=toLong(_n3);
    uint16_t chk;
    uint8_t len;
    len =12;
    chk = HEADER + len + RAW_BARO +sum4byte(n1)+sum4byte(n2)+sum4byte(n3);
    //Serial3.print("ATT");
   // Serial3.println(n1);
    sendbyte(HEADER); //hdr
    sendbyte(len);    //len
    sendbyte(RAW_BARO); //msg id
    send4byte(n1); //data1
    send4byte(n2); //data2
    send4byte(n3); //data3
    send2byte(chk); //chk
}
static void send_cont_compass(int n1,int n2,int n3,int n4,int n5,int n6) {
  

    uint16_t chk;
    uint8_t len;
    len =12;
    chk = HEADER + len + CONT_COMPASS +sum2byte(n1)+sum2byte(n2)+sum2byte(n3)+sum2byte(n4)+sum2byte(n5)+sum2byte(n6);
    //Serial3.print("ATT");
   // Serial3.println(n1);
    sendbyte(HEADER); //hdr
    sendbyte(len);    //len
    sendbyte(CONT_COMPASS); //msg id
    send2byte(n1); //data1
    send2byte(n2); //data2
    send2byte(n3); //data3
    send2byte(n4); //data1
    send2byte(n5); //data2
    send2byte(n6); //data3
    send2byte(chk); //chk
}
static void send_cont_pid(uint16_t n1,uint16_t n2,uint16_t n3,uint16_t n4,uint16_t n5,uint16_t n6,uint16_t n7,uint16_t n8,uint16_t n9,uint16_t n10,uint16_t n11,uint16_t n12) {
  
    uint16_t chk;
    uint8_t len;
    len =24;
    chk = HEADER + len + CONT_PID +sum2byte(n1)+sum2byte(n2)+sum2byte(n3)+sum2byte(n4)+sum2byte(n5)+sum2byte(n6)+sum2byte(n7)+sum2byte(n8)+sum2byte(n9)+sum2byte(n10)+sum2byte(n11)+sum2byte(n12);
    //Serial3.print("ATT");
   // Serial3.println(n1);
    sendbyte(HEADER); //hdr
    sendbyte(len);    //len
    sendbyte(CONT_PID); //msg id
    send2byte(n1); //data1
    send2byte(n2); //data2
    send2byte(n3); //data3
    send2byte(n4); //data1
    send2byte(n5); //data2
    send2byte(n6); //data3
    send2byte(n7); //data1
    send2byte(n8); //data2
    send2byte(n9); //data3
    send2byte(n10); //data1
    send2byte(n11); //data2
    send2byte(n12); //data3
    send2byte(chk); //chk
}
static void alt_send(float _n1,float _n2) {
  
    long n1=toLong(_n1);
    long n2=toLong(_n2);
    uint16_t chk;
    uint8_t len;
    len =8;
    chk = HEADER + len + ALTITUDE +sum4byte(n1)+sum4byte(n2);
    //Serial3.print("ATT");
   // Serial3.println(n1);
    sendbyte(HEADER); //hdr
    sendbyte(len);    //len
    sendbyte(ALTITUDE); //msg id
    send4byte(n1); //data1
    send4byte(n2); //data2
    send2byte(chk); //chk
}
//static void gps_send(long n1,long n2) {
//    uint16_t chk;
//    uint8_t len;
//    len =8;
//    chk = HEADER + len + GPS +sum4byte(n1)+sum4byte(n2);
//    sendbyte(HEADER); //hdr
//    sendbyte(len);    //len
//    sendbyte(GPS); //msg id
//    send4byte(n1); //data1
//    send4byte(n2); //data2
//    send2byte(chk); //chk
//}

static void gps_send(int n1,int n2) {
  
    uint16_t chk;
    uint8_t len;
    len =4;
    chk = HEADER + len + GPS +sum2byte(n1)+sum2byte(n2);
    //Serial3.print("ATT");
   // Serial3.println(n1);
    sendbyte(HEADER); //hdr
    sendbyte(len);    //len
    sendbyte(GPS); //msg id
    send2byte(n1); //data1
    send2byte(n2); //data2
    send2byte(chk); //chk
}
static void nav_send(int n1,int n2) {
  
    uint16_t chk;
    uint8_t len;
    len =4;
    chk = HEADER + len + NAV_DATA +sum2byte(n1)+sum2byte(n2);
    //Serial3.print("ATT");
   // Serial3.println(n1);
    sendbyte(HEADER); //hdr
    sendbyte(len);    //len
    sendbyte(NAV_DATA); //msg id
    send2byte(n1); //data1
    send2byte(n2); //data2
    send2byte(chk); //chk
}
static void flt_send(uint8_t _type,float _f[],uint8_t _len) {
    
    uint16_t sum_data=0;
    uint16_t chk;
    uint8_t len=_len*4;
    
    sendbyte(HEADER); //hdr
    sendbyte(len);    //len
    sendbyte(_type); //msg id
    for(int i = 0;i<_len;i++) {
        float ff = _f[i];
        long dd = toLong(ff);
        
        send4byte(dd);
        sum_data += sum4byte(dd);
    }
    
    chk = HEADER + len + _type +sum_data;
    send2byte(chk); //chk
    //Serial3.print("FLT");
    //Serial3.println();
}


//static void Send_data() {
//  att=true;
//  gps=true;
//    if(att) att_send( SayFloat(f1),SayFloat(f2),SayFloat(f3) ); //send attitude
//    if(gps) gps_send( SayFloat(_g1),SayFloat(_g2) );
////    f1+=1.5;
////    f2+=2;
////    f3+=3.2;
//}


//
//void Serial3Event() {
//  
//     while (Serial3.available()) {
//        uint8_t hdr = Serial3.read();
//        //mySerial3.println(hdr);
//        if (hdr == HEADER) {
//            uint8_t len = Serial3.read();
//            uint8_t msg_id = Serial3.read();
//            uint8_t chk1[2];
//            uint8_t pay[len];
//            uint16_t sum_pay=hdr+len+msg_id;
//            for (int i=0 ; i<len ;i++) {
//                pay[i]=Serial3.read();
//                sum_pay+=pay[i];
//            }
//            chk1[0] = Serial3.read();
//            chk1[1] = Serial3.read();
//            
//            uint16_t chk = *(uint16_t*)(chk1);
//            
//            if (chk == sum_pay) {
////               mySerial3.println(" chk ok");
//                if (msg_id == CMD_TAKEOFF) {
//                    uint8_t h;
//                    h = pay[0];
//                   //mySerial3.print("takeoff to : ");
//                   //mySerial3.println(h);
//                }
//                else if(msg_id == DATA_REQUEST) {
//                    //mySerial3.print("get data request : ");
//                    //mySerial3.println(pay[0]);
//                    switch (pay[0]) {
//                      case DATA_ATT :
//                        att = true;
//                        break;
//                      case DATA_GPS :
//                        gps = true;
//                        break;
//                    }
//                  
//                }
//            }
//        }
//    }
//  
//}
//
//
//



