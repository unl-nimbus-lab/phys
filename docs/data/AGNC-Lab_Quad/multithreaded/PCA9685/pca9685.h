#include "I2C/i2c.h"
#include "math.h"

#ifndef PCA9685_H_
#define PCA9685_H_


#define PCA_DEVICE_ADDR 0x40

#define PCA9685_MODE1 0x00
#define PCA9685_MODE2 0x01
#define PCA9685_PRESCALE 0xFE

#define LED0_ON_L 0x6
#define LED0_ON_H 0x7
#define LED0_OFF_L 0x8
#define LED0_OFF_H 0x9



class pca9685 
{
public:
    pca9685(I2C* i2c);
    ~pca9685();
    int initialize();
    void pwmPulse(uint8_t num, uint16_t on, uint16_t off);
    int pwmFreq(float freq);
    int resetDev();
private:
    I2C* i2c;
};

#endif
