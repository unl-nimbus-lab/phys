int CH_THR;
int RF_ROLL,RF_ROLL_RAW;
int RF_PIT,RF_PIT_RAW;
int RF_YAW,RF_YAW_RAW;
int AUX_1;
int AUX_2;
//int AUX_3;
#define roll_mid 1502
#define pitch_mid 1502
#define yaw_mid 1502
#define THR_MID 1502
float RF_YAW_PLUS_kep=0;
float RF_YAW_PLUS=0;
#define MAXCHANIN 8
#define tarremote 0.062

// FAILSAFE channels
#define FAILSAFE { 1350,1500,1500,1500, 1000, 1000, 1500, 1500 }
#define CENTER 1500 // us

// Channel Name with receiver (or order of the PPMSUM
#define ROLL 0
#define PITCH 1
#define THR 2
#define YAW 3
#define AUX1 4
#define AUX2 5
//#define AUX3 6
uint16_t diff;
volatile uint16_t rcValue[8] = {1502, 1502, 1502, 1502, 1502, 1013, 1502, 1502}; // interval [1000;2000]
volatile uint16_t last = 0;
volatile uint8_t chan = 0;



static void rx_update()
{
    CH_THR       = rcValue[THR];
    RF_ROLL_RAW  = rcValue[ROLL];
    RF_PIT_RAW   = rcValue[PITCH];
    RF_YAW_RAW   = rcValue[YAW];
    AUX_1 = rcValue[AUX1];
    AUX_2 = rcValue[AUX2];
//    AUX_3 = rcValue[AUX3];
    RF_ROLL = RF_ROLL + (RF_ROLL_RAW - RF_ROLL)*0.02/tarremote;
    RF_PIT  = RF_PIT + (RF_PIT_RAW - RF_PIT)*0.02/tarremote;
    RF_YAW  = RF_YAW + (RF_YAW_RAW - RF_YAW)*0.02/tarremote;
}
void rx_initialize()
{
    TCCR5A =((1<<WGM50)|(1<<WGM51));
    TCCR5B = ((1<<WGM53)|(1<<WGM52)|(1<<CS51)|(1<<ICES5)|(1<<ICNC5));
    OCR5A = 40000; // 0.5us per timertick
    TIMSK5 |= (1<<ICIE5); // activate input capture for PL1
  delay(100);
  rx_update();     //---GET AUX_2
}



///////////////////////////// ISR
ISR(TIMER5_CAPT_vect) { //this ISR is common to every receiver channel, it is call everytime a change state occurs on a digital pin [D2-D7]
        uint16_t now;
        now = micros();
        diff = now - last;
        last = now;
        if(diff>3000) chan = 0;
        else {
          if(900<diff && diff<2000 && chan<MAXCHANIN) {   //Only if the signal is between these values it is valid, otherwise the failsafe counter should move up
            rcValue[chan] = diff;
          }
        chan++;
        }
  }

