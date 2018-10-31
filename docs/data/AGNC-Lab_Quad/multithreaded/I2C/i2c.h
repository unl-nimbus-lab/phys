#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <iostream>
#include <string.h>
#include <errno.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <linux/i2c-dev.h>
#include <stdint.h>
#include <math.h>
#include <pthread.h>

#ifndef I2C_H_
#define I2C_H_

#ifndef TRUE
#define TRUE	(1==1)
#define FALSE	(0==1)
#endif

class I2C
{
public:
    I2C(char busAddr);
    ~I2C();
    void openConnection();
    int selectDevice(int addr);
    int writeToDevice(int devAddr, int reg, int val);
    void writeByte(int devAddr, uint8_t DATA_REGADD, uint8_t data);
    uint8_t readByte(int devAddr, uint8_t Data_REGADD);
    uint8_t readMoreBits(int devAddr, uint8_t DATA_REGADD, uint8_t length, uint8_t startBit);
    void writeMoreBits(int devAddr, uint8_t DATA_REGADD, uint8_t data, uint8_t length, uint8_t startBit);
    int16_t readWord(int devAddr, uint8_t MSB, uint8_t LSB);
    int writeData(int devAddr, void* data, int num);
    //int writeData(uint8_t data[], int num);
    //int writeData(int* data, int num);

    
    static int8_t readBit(uint8_t devAddr, uint8_t regAddr, uint8_t bitNum, uint8_t *data, uint16_t timeout=I2C::readTimeout);
    static int8_t readBitW(uint8_t devAddr, uint8_t regAddr, uint8_t bitNum, uint16_t *data, uint16_t timeout=I2C::readTimeout);
    static int8_t readBits(uint8_t devAddr, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t *data, uint16_t timeout=I2C::readTimeout);
    static int8_t readBitsW(uint8_t devAddr, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint16_t *data, uint16_t timeout=I2C::readTimeout);
    static int8_t readByte(uint8_t devAddr, uint8_t regAddr, uint8_t *data, uint16_t timeout=I2C::readTimeout);
    static int8_t readWord(uint8_t devAddr, uint8_t regAddr, uint16_t *data, uint16_t timeout=I2C::readTimeout);
    static int8_t readBytes(uint8_t devAddr, uint8_t regAddr, uint8_t length, uint8_t *data, uint16_t timeout=I2C::readTimeout);
    static int8_t readWords(uint8_t devAddr, uint8_t regAddr, uint8_t length, uint16_t *data, uint16_t timeout=I2C::readTimeout);

    static bool writeBit(uint8_t devAddr, uint8_t regAddr, uint8_t bitNum, uint8_t data);
    static bool writeBitW(uint8_t devAddr, uint8_t regAddr, uint8_t bitNum, uint16_t data);
    static bool writeBits(uint8_t devAddr, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t data);
    static bool writeBitsW(uint8_t devAddr, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint16_t data);
    static bool writeByte(uint8_t devAddr, uint8_t regAddr, uint8_t data);
    static bool writeWord(uint8_t devAddr, uint8_t regAddr, uint16_t data);
    static bool writeBytes(uint8_t devAddr, uint8_t regAddr, uint8_t length, uint8_t *data);
    static bool writeWords(uint8_t devAddr, uint8_t regAddr, uint8_t length, uint16_t *data);

    static uint16_t readTimeout;

private:
    int m_fd;
    char m_busAddr;
    static pthread_mutex_t I2C_Mutex;
};

#endif
