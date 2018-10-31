#include "pca9685.h"


pca9685::pca9685(I2C* i2c)
{
    this->i2c = i2c;
}
pca9685::~pca9685()
{
    resetDev();
}

int pca9685::initialize()
{
    /*
    resetDev();
  
    uint8_t msg[2];


    msg[0] = 0xFD;
    msg[1] = 0b00010000;
    i2c->writeData(PCA_DEVICE_ADDR, msg, 2);

    msg[0] = 0x00;
    msg[1] = 0b00100000;
    i2c->writeData(PCA_DEVICE_ADDR, msg, 2); 


    msg[0] = 0x01;
    msg[1] = 0b00000110;
    i2c->writeData(PCA_DEVICE_ADDR, msg, 2);


    msg[0] = 0x00;
    msg[1] = 0b00000000; //0b00010000
    i2c->writeData(PCA_DEVICE_ADDR, msg, 2);
    usleep(500);

    msg[0] = 0xFE;
    uint8_t temp = 3.0;
    msg[1] = temp;
    i2c->writeData(PCA_DEVICE_ADDR, msg, 2);

    msg[0] = 0x00;
    msg[1] = 0b00100000;
    i2c->writeData(PCA_DEVICE_ADDR, msg, 2);

    usleep(100000);
    pwmPulse(0, 0, 2048);
    usleep(100000);// 0.5 sec delay
    pwmPulse(1, 0, 2048);
    usleep(100000);// 0.5 sec delay
    pwmPulse(2, 0, 2048);
    usleep(100000);// 0.5 sec delay
    pwmPulse(3, 0, 2048);
    usleep(100000);
    */

    resetDev();

    float freq = 500;

    if(pwmFreq(freq)!=1)
        printf("ERROR: setting pwm frequency\n");
    else
        printf("Frequency set to %f Hz.\n", freq);

    i2c->selectDevice(PCA_DEVICE_ADDR);

    int pl1 = 2048; 

    pwmPulse(0, 0, pl1);
    usleep(100000);// 0.5 sec delay
    pwmPulse(1, 0, pl1);
    usleep(100000);// 0.5 sec delay
    pwmPulse(2, 0, pl1);
    usleep(100000);// 0.5 sec delay
    pwmPulse(3, 0, pl1);
    usleep(100000);// 0.5 sec delay

    resetDev();


    return 1;

    }

void pca9685::pwmPulse(uint8_t num, uint16_t on, uint16_t off) 
{

    uint8_t pulseData[5];
    pulseData[0] = LED0_ON_L+4*num;
    pulseData[1] = on;
    pulseData[2] = on>>8;
    pulseData[3] = off;
    pulseData[4] = off>>8;

    //Probably don't need this
    //i2c->selectDevice(PCA_DEVICE_ADDR);

    if(i2c->writeData(PCA_DEVICE_ADDR, pulseData, 5)!=5)
        printf("ERROR in pwmPulse: write\n");
}

int pca9685::pwmFreq(float freq) 
{

    float prescaleval = 25000000;
    prescaleval /= 4096;
    prescaleval /= freq;
    prescaleval -= 1;
    uint8_t prescale = floor(prescaleval + 0.5);

    uint8_t oldmode = i2c->readByte(PCA_DEVICE_ADDR, PCA9685_MODE1);
    //printf("oldmode = %d\n", oldmode);// = 1 for resetDev(fh) in main()

    uint8_t newmode = (oldmode&0x7F) | 0x10; // sleep

    i2c->writeByte(PCA_DEVICE_ADDR, PCA9685_MODE1, newmode); // go to sleep

    i2c->writeByte(PCA_DEVICE_ADDR, PCA9685_PRESCALE, prescale); // set the prescaler

    i2c->writeByte(PCA_DEVICE_ADDR, PCA9685_MODE1, oldmode);

    i2c->writeByte(PCA_DEVICE_ADDR, PCA9685_MODE1, oldmode | 0xa1);// MODE1 to auto increment

    return 1;
}

int pca9685::resetDev() 
{
    // if(i2c->selectDevice(PCA9685_MODE1)<0) 
    //     printf("ERROR in reset: ioctl\n");
    int reset = 0x06;
    if(i2c->writeData(PCA9685_MODE1, &reset, 1)!=1)
        printf("ERROR in reset: write\n");


    //i2c->selectDevice(PCA_DEVICE_ADDR);
    i2c->writeByte(PCA_DEVICE_ADDR, PCA9685_MODE1, 0x0);
    usleep(500);
    return 1;
}
