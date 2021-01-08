/*
Operation     Output Range                            Output Type                 int8_ts per Element      Output Class
uint8           0 to 255                              Unsigned 8-bit integer             1                  uint8
uint16          0 to 65,535                           Unsigned 16-bit integer            2                  uint16
uint32          0 to 4,294,967,295                    Unsigned 32-bit integer            4                  uint32
uint64          0 to 18,446,744,073,709,551,615       Unsigned 64-bit integer            8                  uint64
*/

//Configable Flight mode-----------------------------------------------------------------
#define mode_1 STABILIZE_MODE
#define mode_2 ALT_MODE
#define mode_3 VEL_HOLD
#define mode_4 STABILIZE_MODE
#define mode_5 TAKE_OFF
#define mode_6 ALT_MODE
//-------------------------------------------------------------------------------------
//#define COMMUNICATE_VIA_WIRE                  //----- for use FTDI direct connect to 
#define USE_PROTOCAL                          //----- for use WIRELESS communication
//-------------------------------------------------------------------------------------
#define DISABLE_INTERNAL_MAG
//#define YAW_ANGLE_CONTROL
//--------------------------------------------------------------------------------------
//#define MAF_MODE //or comment this line to use average
//--------------------------------------------------------------------------------------
#define TEST_NUM_SENSOR



//Add when new mode come
#define STABILIZE_MODE 0
#define ALT_MODE 1
#define TAKE_OFF 3
#define OTHER_MODE 2
#define VEL_HOLD 4
#define GUIDE_MODE 5

#define QUAD_MASS 1.50f
#define ONE_G 9.80655
#define PROTECT_PROP_ANGLE 1.0

#define instability_fix 1
#define COMPASS_CALIBRATION




//to hover in ALT MODE
#define HOVERING_THROTTLE 1450
int HOVERING_THROTTLE_now = 1450;
//to check propeller will stop
#define CH_THR_threshold 1100


unsigned long loopt,loops;


////baro glitch configuration---------------------------------------------------------------------------------------
//bool _baro_healthy = false;
#define _dist_ok_cm 50.0f
#define baro_glitch_enabled 1 
#define _accel_max_cmss 150.0f
////---------------------------------------------------------------------------------------------------------------










float G_Dt = 0.002; 

//--------------EEPROM HANDLING-------------------------------------
////////////////////////////////////////////////////////////////////
//magnetometer calibration constants; use the Calibrate example from
// the Pololu library to find the right values for your board
int16_t M_X_MIN = -493;    //       ----- -416/517	-322/590	-394/480     internal compass
int16_t M_X_MAX = 421;     //       ----- -493,421	-356,568	-396,399     external compass
int16_t M_Y_MIN = -356;    //
int16_t M_Y_MAX = 568;     //
int16_t M_Z_MIN = -396;    //
int16_t M_Z_MAX = 399;     //

bool compass_calibrate = false;



//PID-------------Rate
uint16_t Kp_rateRoll=70.0  ; //70.0  try smaller 40 is fine but not strong pid
uint16_t Ki_rateRoll=1 ;  //0.25
uint16_t Kd_rateRoll=5.00  ; //5.00

uint16_t Kp_ratePitch=70.0;
uint16_t Ki_ratePitch=1;   //0.25  test --> 1
uint16_t Kd_ratePitch=5.00;

uint16_t Kp_rateYaw=100.0 ; //120 --> hot
uint16_t Ki_rateYaw=15.0;  //20
uint16_t Kd_rateYaw=7.0  ; //7.0

//PID--------------Stable
uint16_t Kp_levelRoll=10.00 ; //10.00
uint16_t Ki_levelRoll=0.00;
uint16_t Kd_levelRoll=0.00;

uint16_t Kp_levelPitch=10.00;  //10.00
uint16_t Ki_levelPitch=0.00;
uint16_t Kd_levelPitch=0.00;

uint16_t Kp_levelYaw=7.00;  //4.50  ->  7.00
uint16_t Ki_levelYaw=0.00;
uint16_t Kd_levelYaw=0.00;

//-------------------------



//-------------------------
#define YAW_I_MAX 150
//#define YAW_I_OFFSET 110
#define YAW_I_OFFSET 0


//------------------------------------------EXPERIMENT ONBOARD VELOCITY------------------------------ 
//PID vel control---------
#define Kp_vel 0.3
#define Ki_vel 0
float vx_theory=0,vx_theory_ef,vx_fillered; // ef=earth-frame
float vy_theory=0,vy_theory_ef,vy_fillered; 
//float data_buffer_vx=0,data_buffer_vy=0;
//int8_t num_vx=0,num_vy=0;
#define _kdrag 0.350f  //0.30f
#define kdrag _kdrag/QUAD_MASS


//PID pos control

float hx,hy,Dhx_b,Dhy_b;
#define Kp_pos 1
#define Ki_pos 1
float des_hx=0,des_hy=0;


int thr=0;
int count_time=0;
#define TIME_ARM 300
#define tar 0.01
int8_t armed;


//---------------------LED SETUP PIN-----------------------------------

#define RED_LED 13
#define YELLOW_LED 31
#define GREEN_LED 30
// static void LED_SETUP() {
//   pinMode(RED_LED, OUTPUT); //LED A RED
//   pinMode(YELLOW_LED, OUTPUT); //LED B YELLOW
//   pinMode(GREEN_LED, OUTPUT); //LED C GREEN
// }
//-------------------TASK WINDOW---------------------------------------

#define TASK_100HZ 1
#define TASK_50HZ 2
#define TASK_33HZ 3
#define TASK_25HZ 4
#define TASK_15HZ 6
#define TASK_10HZ 10
#define TASK_1HZ 100


///-----------------COMMUNICATION--------------------------------------

//-----------------------------

// #ifdef COMMUNICATE_VIA_WIRE
//   #define begin_serial Serial.begin(115200);Serial3.begin(57600);
//   #define say(x)      Serial.print(x);
//   #define saycomma(x) Serial.print(x);Serial.print(","); 
//   #define saytab(x)   Serial.print(x);Serial.print("\t"); 
//   #define sayend(x)   Serial.print(x);Serial.print("\n"); 
// #else  
//   #define begin_serial Serial.begin(115200);Serial3.begin(57600);
//   #define say(x)      Serial3.print(x);
//   #define saycomma(x) Serial3.print(x);Serial3.print(","); 
//   #define saytab(x)   Serial3.print(x);Serial3.print("\t"); 
//   #define sayend(x)   Serial3.print(x);Serial3.print("\n"); 
// #endif


//void SerialHandle() {
//  begin_serial
//#if defined(TEST_NUM_SENSOR)
//#ifndef COMMUNICATE_VIA_WIRE
//  Serial.begin(115200);
//#endif
//#endif
//}

///-----------------END  COMMUNICATION-----------------------------------------



//-----------------Remote Variable -------------------------------------------
int CH_THR;
int RF_ROLL,RF_ROLL_RAW;
int RF_PIT,RF_PIT_RAW;
int RF_YAW,RF_YAW_RAW;
int AUX_1;
int AUX_2 = 1013;
//int AUX_3;
#define roll_mid 1502
#define pitch_mid 1502
#define yaw_mid 1502
#define THR_MID 1502
float RF_YAW_PLUS_kep=0;
float RF_YAW_PLUS=0;

//----------------------GPS Variable-------------------------------------------
struct AP_GPS
{
     long lat;
     long lng;
     float alt;
     unsigned long last_fix_time_ms;
}_GPS;
Location ahrs_home;
//float  _GPS.lat, _GPS.lng;
//float lat_home,long_home;
float GPS_GroundSpeed,GPS_Heading;
int GPS_fix,GPS_hDOP,GPS_numSAT;
float GPS_vel_y_fillered,GPS_vel_x_fillered;

//--------------------AUTO MODE VARIABLE---------------------------------------
bool AUTO_MODE = false;
float desire_yaw = 0;
#define YAW_TURN_RATIO 0.0025  // max per 1 rad/s
#define YAW_TURN_RATE YAW_TURN_RATIO*1.5
#define YAW_TURN_RATE_AUTO YAW_TURN_RATIO*0.3
#define SPAN_OF_YAW_TURNING 0.08 //number rad of switching between pid angle and pid rate
//--------------------END AUTO MODE VARIABLE-----------------------------------
#define applyDeadband(value, deadband)  \
  if(abs(value) < deadband) {           \
    value = 0;                          \
  } else if(value > 0){                 \
    value -= deadband;                  \
  } else if(value < 0){                 \
    value += deadband;                  \
  }
#define wrap_180(x) (x < -180 ? x+360 : (x > 180 ? x - 360: x))
#define wrap_pi(x) (x < -3.14 ? x+6.28 : (x > 3.14 ? x - 6.28: x))
#define HALF_M_PI 1.570796

//remote alt-hold up
float z_up=0;
float takeoff_height = 0;
bool flag_takeoff_processing=false;
bool flag_takeoff_done=false;


int FLIGHT_MODE_CHECK = 0;
static void FLIGHT_MODE(int signal){

  if        (signal <= 1150)                    FLIGHT_MODE_CHECK=mode_1; 
  else if   (signal > 1150 && signal <= 1380)   FLIGHT_MODE_CHECK=mode_2; 
  else if   (signal > 1380 && signal <= 1500)   FLIGHT_MODE_CHECK=mode_3; 
  else if   (signal > 1500 && signal <= 1600)   FLIGHT_MODE_CHECK=mode_4; 
  else if   (signal > 1600 && signal <= 1700)   FLIGHT_MODE_CHECK=mode_5; 
  else                                          FLIGHT_MODE_CHECK=mode_6; 
  
  //take off done then goto ALT-mode alway although that signal is take off mode----------------
  if( FLIGHT_MODE_CHECK == TAKE_OFF && flag_takeoff_done==true ) FLIGHT_MODE_CHECK = ALT_MODE;
  //--------------------------------------------------------------------------------------------
}
static int8_t AUX_2_STATUS() { saytab("AUX_2 val=");sayend(AUX_2); return ( (AUX_2>1200) ? 1:0); } 
static bool IS_USE_ALT_HOLD() { return (FLIGHT_MODE_CHECK == ALT_MODE || FLIGHT_MODE_CHECK == VEL_HOLD || FLIGHT_MODE_CHECK == TAKE_OFF);}

static void say_mode(){
  switch(FLIGHT_MODE_CHECK)
  {
    case STABILIZE_MODE : sayend("current_mode = Stabilize"); break;
    case ALT_MODE :       sayend("current_mode = ALT");       break;
    case TAKE_OFF :       sayend("current_mode = TAKE OFF!"); break;
    case OTHER_MODE :     sayend("current_mode = OTHER");     break;
    case VEL_HOLD :       sayend("current_mode = VEL_HOLD");     break;
    default : break;
  }
}




//------------------------------------------------TASK VARIABLES---------------------------------------------
// sub loop time variable
unsigned long oneHZpreviousTime = 0;
unsigned long tenHZpreviousTime = 0;
unsigned long lowPriorityTenHZpreviousTime = 0;
unsigned long lowPriorityTenHZpreviousTime2 = 0;
unsigned long fiftyHZpreviousTime = 0;
unsigned long fifteenHZpreviousTime = 0;
unsigned long thitythreeHZpreviousTime = 0;
unsigned long hundredHZpreviousTime = 0;
//unsigned long frameCounter = 0; // main loop executive frame counter

// Main loop variables
unsigned long currentTime = 0;
unsigned long previousTime = 0;
unsigned long sensorPreviousTime = 0;
uint8_t frameCounter = 0;
uint32_t itterations = 0;


// Sensor task variable
unsigned long _barolast_update = 0;


//------------------------------------------------END  TASK VARIABLES---------------------------------------------



//-----------------------------------------------CONTROL VARIABLES------------------------------------------------
float roll_I_rate;
float roll_D_rate;
float err_roll_rate;
float err_roll_ant_rate;

float pitch_I_rate;
float pitch_D_rate;
float err_pitch_rate;
float err_pitch_ant_rate;

float yaw_I_rate;
float yaw_D_rate;
float err_yaw_rate=0;
float err_yaw_ant_rate=0;


float roll_I_level;
float roll_D_level;
float roll_D_level_test;
float err_roll_level=0;
float err_roll_ant_level=0;

float pitch_I_level;
float pitch_D_level;
float err_pitch_level=0;
float err_pitch_ant_level=0;

float yaw_I_level;
float yaw_D_level;
float err_yaw_level=0;
float err_yaw_ant_level=0;


float last_input_roll=0;
float last_input_pitch=0;
float last_input_yaw=0;

float control_roll=0.0;          
float control_pitch=0.0;
float control_yaw=0.0;

//float holdAltitude=0.0;
//float altitude=0.0;
//float altitude_baro=0.0;
float factor_rotate=1;

//----------------------------------------------END CONTROL VARIABLE----------------------------------

//---------------------------------------------IMU FILERED VARIABLE---------------------------------
//------Low-pass filter gyro acc
//float G_f[3]={0,0,0};

float Acc_f[3]={0,0,0};
//float Acc_ft[3]={0,0,0};
//------g3_trim test-----------
static float g3_trim;
//--------------------------------------------END IMU FILERED VARIABLE-------------------------------
//--------------------------------------------STATUS VARIABLE----------------------------------------
//bool baro_healthy = true;
//float press_last = 0;

//LANDING CHK--------------------
bool Landed = true;
#define LAND_CK_MOTOR 1100

//IMU---------------------------
// Axis definitions
#define XAXIS 0
#define YAXIS 1
#define ZAXIS 2
float                ahrs_r_trim = 0;
float                ahrs_p_trim = 0;
float                ahrs_y_trim = 0;
float                alt_trim = 0;
bool              flag_trim = true;
int                  count_for_trim=0;
int                   num_readings = 100;
float                 x_accel = 0;
float                 y_accel = 0;
float                 z_accel = 0;
float                 x_gyro = 0;
float                 y_gyro = 0;
float                 z_gyro = 0;

float gyro[3];
float accel[3];

float gyroScaleFactor = radians(1000.0 / 32768.0);
float accelScaleFactor = 9.80665 / 8192.0;   
uint16_t sensors_detected = 0x00;

uint8_t gyroSamples = 0;
uint8_t accelSamples = 0; 


int16_t gyroRaw[3];
float gyroSum[3];

int16_t accelRaw[3];
float accelSum[3];

int16_t gyro_offset[3];
int16_t accel_offset[3];  

//ahrs-------------------------

float roll_dot;
float pitch_dot;


////////////////////////////////////////////
#define Kp 0.22 //0.15  //0.2 1.62	// proportional gain governs rate of convergence to accelerometer/magnetometer
#define Ki 0.072 //0.072 0.096 0.05	// integral gain governs rate of convergence of gyroscope biases
#define Kp_mag 0.25  //0.22
#define Ki_mag 0.112 //0.142
float exInt , eyInt , ezInt ;	// scaled integral error
float q0, q1 , q2 , q3 ;	// quaternion elements representing the estimated orientation
float ahrs_p,ahrs_r,ahrs_y;
float Heading = 0.0;
float setHeading = 0.0;
float previousEx = 0.0;
float previousEy = 0.0;
float previousEz = 0.0;


//--------------------AlTITUDE---------------------------------
unsigned long now1=0;
float g1 ,g2,g3;
float qc0,qc1,qc2,qc3;
float w,x,y,z;
///////////////////////////////////////////////
//direction cosine matrix (DCM)   Rotated Frame to Stationary Frame ZYX



float DCM00 = 1.0;
float DCM01 = 0.0;
float DCM02 = 0.0;
float DCM10 = 0.0;
float DCM11 = 1.0;
float DCM12 = 0.0;
float DCM20 = 0.0;
float DCM21 = 0.0;
float DCM22 = 1.0;
//float DCM23 = 1.0;
float cos_rollcos_pitch = 1.0;

//trig
float cos_roll = 0.0;
float sin_roll = 0.0;
float cos_pitch = 0.0;
float sin_pitch = 0.0;
float cos_yaw = 0.0;
float sin_yaw = 0.0;


Vector3f accel_ef; //DCM untest but z has been in use
Vector3f acc_ef;  //test nav

float accrX_Earth = 0.0;
float accrY_Earth = 0.0;
float accrZ_Earth = 0.0;
float accrZ_Earthf = 0.0;

Vector3f accel_correction_hbf;
static void setup_home_position(void) ;
static void set_velocity_xy(float x, float y);
//-------------------------------EEPROM----------------------------------------------------------------
// #include <EEPROM.h>
// #define TD_EEPROM_BASE 0x0A
// #define TD_EEPROM_SIGNATURE 0x13


// static uint8_t location; // assuming ordered reads

// static void eeprom_read_var(uint8_t size, int8_t * var) {
//   for(uint8_t i = 0; i<size; i++) {
//     var[i] = EEPROM.read(location + i);
//   }
//   location += size;
// }

// static void calLoad_PID() {
//   if(EEPROM.read(TD_EEPROM_BASE) == TD_EEPROM_SIGNATURE) { // check if signature is ok so we have good data
//     location = TD_EEPROM_BASE + 1; // reset location
    
//   //PID------------------------------------------------
//     eeprom_read_var(sizeof(Kp_levelRoll), (int8_t *) &Kp_levelRoll);
//     eeprom_read_var(sizeof(Kp_rateRoll), (int8_t *) &Kp_rateRoll);
//     eeprom_read_var(sizeof(Ki_rateRoll), (int8_t *) &Ki_rateRoll);
//     eeprom_read_var(sizeof(Kd_rateRoll), (int8_t *) &Kd_rateRoll);
//     eeprom_read_var(sizeof(Kp_levelPitch), (int8_t *) &Kp_levelPitch);
//     eeprom_read_var(sizeof(Kp_ratePitch), (int8_t *) &Kp_ratePitch);
//     eeprom_read_var(sizeof(Ki_ratePitch), (int8_t *) &Ki_ratePitch);
//     eeprom_read_var(sizeof(Kd_ratePitch), (int8_t *) &Kd_ratePitch);
//     eeprom_read_var(sizeof(Kp_levelYaw), (int8_t *) &Kp_levelYaw);
//     eeprom_read_var(sizeof(Kp_rateYaw), (int8_t *) &Kp_rateYaw);
//     eeprom_read_var(sizeof(Ki_rateYaw), (int8_t *) &Ki_rateYaw);
//     eeprom_read_var(sizeof(Kd_rateYaw), (int8_t *) &Kd_rateYaw);
//   }
// }
// static void PID_UPDATE() {
//   int int8_t_changed = 0;
//   const uint8_t eepromsize = sizeof(uint16_t) * 12 /*+ sizeof(int) * 6*/;
//   uint8_t data[eepromsize],data_old[eepromsize];
//   while(Serial3.available() < eepromsize) ; // wait until all calibration data are receive
//   EEPROM.write(TD_EEPROM_BASE, TD_EEPROM_SIGNATURE);
//   for(uint8_t i = 1; i<(eepromsize)+1; i++) {
//     data_old[i-1] = EEPROM.read(TD_EEPROM_BASE + i);
//     data[i-1] = Serial3.read();
//     //Serial.println((uint8_t)c);
//     if(data_old[i-1] != data[i-1]) 
//     {
//       int8_t_changed++;
//       EEPROM.write(TD_EEPROM_BASE + i,data[i-1] );
//     }
//   }
//   Serial.print("Changed int8_t == ");Serial.println(int8_t_changed);
                  
// //      Serial.println(toUint(data[0]));
// //      Serial.println(toUint(data[4]));
// //      Serial.println(toUint(data[8]));
//   // toggle LED after calibration store.
//   digitalWrite(RED_LED, HIGH);
//   delay(1000);
//   digitalWrite(RED_LED, LOW);
  
  
  
  
  
//   calLoad_PID();
  
//                   saycomma(Kp_levelRoll);                
//                   saycomma(Kp_rateRoll  );         
//                   saycomma(Ki_rateRoll  );         
//                   saycomma(Kd_rateRoll  );         
//                   saycomma(Kp_levelPitch);          
//                   saycomma(Kp_ratePitch );         
//                   saycomma(Ki_ratePitch );          
//                   saycomma(Kd_ratePitch );          
//                   saycomma(Kp_levelYaw  );         
//                   saycomma(Kp_rateYaw   );                
//                   saycomma(Ki_rateYaw   );               
//                   sayend  (Kd_rateYaw   );
//   digitalWrite(RED_LED, HIGH);
//   delay(1000);
//   digitalWrite(RED_LED, LOW);
// }

// static void RECIEVE_COMMAND_FROM_GROUND_STATION() {
//                  uint8_t cmd = Serial3.read();
//                 if(cmd==0x13)  {
//                    Serial.println("SEND COMPASS CONST DATA");
//                    send_cont_compass(M_X_MIN,M_X_MAX,M_Y_MIN,M_Y_MAX,M_Z_MIN,M_Z_MAX);
//                 }else if(cmd==0x14) {
//                   Serial.println("PID SEND");
//                   send_cont_pid(Kp_levelRoll, Kp_rateRoll, Ki_rateRoll, Kd_rateRoll
//                                ,Kp_levelPitch,Kp_ratePitch,Ki_ratePitch,Kd_ratePitch
//                                ,Kp_levelYaw,  Kp_rateYaw,  Ki_rateYaw,  Kd_rateYaw );
//                 }else if(cmd==0x15) {
//                   Serial.println("PID recieved!");
//                   PID_UPDATE();
//                 }
// }
