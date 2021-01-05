int CH_THR;
int RF_ROLL;
int RF_PIT;
int RF_YAW;
int AUX_1;
int AUX_2;

#define roll_mid 1475
#define pitch_mid 1479
#define yaw_mid 1464
#define THR_MID 1500
float RF_YAW_PLUS_kep=0;
float RF_YAW_PLUS=0;
#define MAXCHANIN 8


// FAILSAFE channels
#define FAILSAFE { 1350,1500,1500,1500, 1000, 1000, 1500, 1500 }
#define CENTER 1500 // us

// Channel Name with receiver (or order of the PPMSUM
#define AIL 1
#define ELE 2
#define THR 0
#define RUD 3
#define AUX 4

static int limit(int mn,int v,int mx)
{
  if (v<mn) return mn;
  if (v>mx) return mx;
  return v;
}

static float limitf(float mn,float v,float mx)
{
  if (v<mn) return mn;
  if (v>mx) return mx;
  return v;
}



static void change(void);
int failsafeChan[]=FAILSAFE;
unsigned long now,diff,last;
byte chan=-1; // -1 invalid



int ichannelIn[MAXCHANIN];
float channelIn[MAXCHANIN];

static void failsafe()
{
  for(byte i=0;i<MAXCHANIN;i++)
  {
    ichannelIn[i]=failsafeChan[i];
    channelIn[i]=(ichannelIn[i]-CENTER)/500.0;
  }
}

void rx_initialize()
{
  failsafe();



  pinMode(A8, INPUT); // 3 is used for esc
  pinMode(A9, INPUT);
  pinMode(A10, INPUT);
  pinMode(A11, INPUT);
  pinMode(A12, INPUT);
  // interrupt on pin change PCINT
  PCICR = 1<<2; // PCINT activated only for PORTK dealing with [A8-A15] PINs

  PCMSK2 |= (1<<0) | (1<<1) | (1<<2) | (1<<3) | (1<<4) | (1<<5) | (1<<6) | (1<<7);

}

static void rx_update()
{
  if (micros()-last>60000L) // 60ms without new data FAILSAFE
  {
    //failsafe();
  }


  for(byte i=0;i<MAXCHANIN;i++)
  {
    channelIn[i]=(limit(1000,ichannelIn[i],2000));  // floaint point value are [-1,1]
  }
  
  CH_THR=channelIn[THR];
  RF_ROLL=channelIn[AIL];
  RF_PIT=channelIn[ELE];
  RF_YAW=channelIn[RUD];
  
  
  //prevent AUX fail to 1000 
  if(channelIn[AUX] !=1000 ) {
  AUX_1=channelIn[AUX];  }
  
  
}



static void change(void)
{
  last=now;
  now=micros();
  diff=now-last;
  if (diff>2300) 
  {
    chan=0;
  }
  else if (chan!=-1 && diff>900 && diff<2200)
    ichannelIn[chan++]=diff;
  else  
    chan=-1; // invalid

}


///////////////////////////// ISR


#define MASKPCINT0 (1<<2)  //throttle
#define MASKPCINT1 (1<<0)  //roll
#define MASKPCINT2 (1<<1)  //pitch
#define MASKPCINT3 (1<<3)  //yaw
#define MASKPCINT4 (1<<4)  //aux
ISR(PCINT2_vect)
{
  static byte newbit,oldbit,changed;
  static unsigned long startIn[5];
  last=now;
  now=micros(); 


  newbit=PINK;


  // This is a new VERY VERY VERY fast method 
  // 1 xor operation 

  changed=newbit^oldbit;

  if (changed&MASKPCINT0)
    if (newbit&MASKPCINT0) startIn[0]=now;
    else ichannelIn[0]=now-startIn[0];

  if (changed&MASKPCINT1)
    if (newbit&MASKPCINT1) startIn[1]=now;
    else ichannelIn[1]=now-startIn[1];

  if (changed&MASKPCINT2)
    if (newbit&MASKPCINT2) startIn[2]=now;
    else ichannelIn[2]=now-startIn[2];

#ifdef MASKPCINT3
  if (changed&MASKPCINT3)
    if (newbit&MASKPCINT3) startIn[3]=now;
    else ichannelIn[3]=now-startIn[3];
#endif

#ifdef MASKPCINT4
  if (changed&MASKPCINT4)
    if (newbit&MASKPCINT4) startIn[4]=now;
    else ichannelIn[4]=now-startIn[4];
#endif

  oldbit=newbit;

}

