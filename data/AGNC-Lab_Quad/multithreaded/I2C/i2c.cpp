#include "i2c.h"


I2C::I2C(char busAddr) 
{
    this->m_busAddr = busAddr;
    pthread_mutex_init(&I2C::I2C_Mutex, NULL);
}


I2C::~I2C()
{
    close(this->m_fd);
    pthread_mutex_destroy(&I2C::I2C_Mutex);
}

void I2C::openConnection() {

    if ((this->m_fd = open("/dev/i2c-3", O_RDWR)) < 0) {
	//error
	exit(1);
    }
}

int I2C::selectDevice(int addr)
{
    return ioctl(m_fd, I2C_SLAVE, addr);
}

int I2C::writeToDevice(int devAddr, int reg, int val)
{
    //lock mutex
    pthread_mutex_lock(&I2C::I2C_Mutex);

    m_fd = open("/dev/i2c-3", O_RDWR);
    if (m_fd < 0) {
        fprintf(stderr, "Failed to open device: %s\n", strerror(errno));
        return(FALSE);
    }

    selectDevice(devAddr);

    uint8_t buf[2];
    buf[0]=reg; buf[1]=val;
    int flag = write(m_fd, buf, 2);

    close(m_fd);

    //unlock mutex
    pthread_mutex_unlock(&I2C::I2C_Mutex);

    return flag;

}


void I2C::writeByte(int devAddr, uint8_t DATA_REGADD, uint8_t data) {

    //lock mutex
    pthread_mutex_unlock(&I2C::I2C_Mutex);

    
    m_fd = open("/dev/i2c-3", O_RDWR);
    if (m_fd < 0) {
        fprintf(stderr, "Failed to open device: %s\n", strerror(errno));
    }

    //switch to device
    selectDevice(devAddr);
    
    uint8_t buffer[2];

    buffer[0] = DATA_REGADD;
    buffer[1] = data;

    if (write(m_fd, buffer, 2) != 2) {
	printf("Can not write data");
    }

    close(m_fd);

    //unlock mutex
    pthread_mutex_unlock(&I2C::I2C_Mutex);

}

uint8_t I2C::readByte(int devAddr, uint8_t DATA_REGADD) 
{
    //lock mutex
    pthread_mutex_lock(&I2C::I2C_Mutex);

    
    m_fd = open("/dev/i2c-3", O_RDWR);
    if (m_fd < 0) {
        fprintf(stderr, "Failed to open device: %s\n", strerror(errno));
        return(FALSE);
    }

    selectDevice(devAddr);

    uint8_t buffer[1];
    buffer[0] = DATA_REGADD;
  
    if (write(m_fd, buffer, 1) != 1) {
	printf("Can not write data");
    }
  
    uint8_t value[1];
  
    if (read(m_fd, value, 1) != 1) {
	printf("Can not read data");
    }
    //unlock mutex
    
    close(m_fd);

    pthread_mutex_unlock(&I2C::I2C_Mutex);

    return value[0];
}

uint8_t I2C::readMoreBits(int devAddr, uint8_t DATA_REGADD, uint8_t length, 
			      uint8_t startBit) 
{
    int8_t temp = readByte(devAddr, DATA_REGADD);
    return (temp >> startBit) % (uint8_t) pow(2, length);
}

void I2C::writeMoreBits(int devAddr, uint8_t DATA_REGADD, uint8_t data, uint8_t length, uint8_t startBit) 
{
    int8_t temp = readByte(devAddr, DATA_REGADD);
    uint8_t bits = 1;
    uint8_t i = 0;

    while (i < length - 1) {
	bits = (bits << 1);
	++bits;
	++i;
    }
    
    temp &= ~(bits << startBit);
    
    temp |= (data << startBit);
    
    writeByte(devAddr, DATA_REGADD, temp);
    
}

int16_t I2C::readWord(int devAddr, uint8_t MSB, uint8_t LSB) 
{

    uint8_t msb = readByte(devAddr, MSB);

    uint8_t lsb = readByte(devAddr, LSB);

  return  ((int16_t)msb << 8) + lsb;
}






//PWM methods

//Lower level write command for PWM
int I2C::writeData(int devAddr, void* data, int num) 
{
    //lock mutex
    pthread_mutex_lock(&I2C::I2C_Mutex);

    m_fd = open("/dev/i2c-3", O_RDWR);
    if (m_fd < 0) {
        fprintf(stderr, "Failed to open device: %s\n", strerror(errno));
        return(FALSE);
    }

    selectDevice(devAddr);

    int flag = write(m_fd, data, num);

    close(m_fd);

    //unlock mutex
    pthread_mutex_unlock(&I2C::I2C_Mutex);

    return flag;
}

//int I2C::writeData(uint8_t data[], int num) 
//{
//    return write(m_fd, data, num);
//}
//
//int I2C::writeData(int* data, int num)
//{
//    return write(m_fd, data, num);
//}



//This is the same as the writeToDevice() function

//void I2C::write8(uint8_t d0, uint8_t d1) {
//    uint8_t d[2];
//    d[0] = (uint8_t)d0;
//    d[1] = (uint8_t)d1;
//    //if(ioctl(I2C_SLAVE, i2c_addr)<0)
//    //    printf("ERROR in write8: ioctl\n");
//    if(write(m_fd, d, 2)!=2)
//        printf("ERROR in write8: write\n");
//   usleep(10);// 10 us pause before any other i2c commands
//}


//This is the same as the readByte() function

//uint8_t I2C::read8(int fh, uint8_t addr) {
//
//    //if(ioctl(fh, I2C_SLAVE, i2c_addr)<0)
//    //    printf("ERROR in read8: ioctl before write()\n");
//    write(m_fd, &addr, 1);
//    usleep(50);
//
//    uint8_t data;
//   //if(ioctl(m_fd, I2C_SLAVE, i2c_addr)<0)
//    //    printf("ERROR in read8: ioctl before read()\n");
//    return read(fh, &data, 1);
//}


//MPU DMP Methods


/** Read a single bit from an 8-bit device register.
 * @param devAddr I2C slave device address
 * @param regAddr Register regAddr to read from
 * @param bitNum Bit position to read (0-7)
 * @param data Container for single bit value
 * @param timeout Optional read timeout in milliseconds (0 to disable, leave off to use default class value in I2C::readTimeout)
 * @return Status of read operation (true = success)
 */
int8_t I2C::readBit(uint8_t devAddr, uint8_t regAddr, uint8_t bitNum, uint8_t *data, uint16_t timeout) {
    uint8_t b;
    uint8_t count = readByte(devAddr, regAddr, &b, timeout);
    *data = b & (1 << bitNum);
    return count;
}

/** Read a single bit from a 16-bit device register.
 * @param devAddr I2C slave device address
 * @param regAddr Register regAddr to read from
 * @param bitNum Bit position to read (0-15)
 * @param data Container for single bit value
 * @param timeout Optional read timeout in milliseconds (0 to disable, leave off to use default class value in I2C::readTimeout)
 * @return Status of read operation (true = success)
 */
int8_t I2C::readBitW(uint8_t devAddr, uint8_t regAddr, uint8_t bitNum, uint16_t *data, uint16_t timeout) {
    uint16_t b;
    uint8_t count = readWord(devAddr, regAddr, &b, timeout);
    *data = b & (1 << bitNum);
    return count;
}

/** Read multiple bits from an 8-bit device register.
 * @param devAddr I2C slave device address
 * @param regAddr Register regAddr to read from
 * @param bitStart First bit position to read (0-7)
 * @param length Number of bits to read (not more than 8)
 * @param data Container for right-aligned value (i.e. '101' read from any bitStart position will equal 0x05)
 * @param timeout Optional read timeout in milliseconds (0 to disable, leave off to use default class value in I2C::readTimeout)
 * @return Status of read operation (true = success)
 */
int8_t I2C::readBits(uint8_t devAddr, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t *data, uint16_t timeout) {
    // 01101001 read byte
    // 76543210 bit numbers
    //    xxx   args: bitStart=4, length=3
    //    010   masked
    //   -> 010 shifted
    uint8_t count, b;
    if ((count = readByte(devAddr, regAddr, &b, timeout)) != 0) {
        uint8_t mask = ((1 << length) - 1) << (bitStart - length + 1);
        b &= mask;
        b >>= (bitStart - length + 1);
        *data = b;
    }
    return count;
}

/** Read multiple bits from a 16-bit device register.
 * @param devAddr I2C slave device address
 * @param regAddr Register regAddr to read from
 * @param bitStart First bit position to read (0-15)
 * @param length Number of bits to read (not more than 16)
 * @param data Container for right-aligned value (i.e. '101' read from any bitStart position will equal 0x05)
 * @param timeout Optional read timeout in milliseconds (0 to disable, leave off to use default class value in I2C::readTimeout)
 * @return Status of read operation (1 = success, 0 = failure, -1 = timeout)
 */
int8_t I2C::readBitsW(uint8_t devAddr, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint16_t *data, uint16_t timeout) {
    // 1101011001101001 read byte
    // fedcba9876543210 bit numbers
    //    xxx           args: bitStart=12, length=3
    //    010           masked
    //           -> 010 shifted
    uint8_t count;
    uint16_t w;
    if ((count = readWord(devAddr, regAddr, &w, timeout)) != 0) {
        uint16_t mask = ((1 << length) - 1) << (bitStart - length + 1);
        w &= mask;
        w >>= (bitStart - length + 1);
        *data = w;
    }
    return count;
}

/** Read single byte from an 8-bit device register.
 * @param devAddr I2C slave device address
 * @param regAddr Register regAddr to read from
 * @param data Container for byte value read from device
 * @param timeout Optional read timeout in milliseconds (0 to disable, leave off to use default class value in I2C::readTimeout)
 * @return Status of read operation (true = success)
 */
int8_t I2C::readByte(uint8_t devAddr, uint8_t regAddr, uint8_t *data, uint16_t timeout) {
    return readBytes(devAddr, regAddr, 1, data, timeout);
}

/** Read single word from a 16-bit device register.
 * @param devAddr I2C slave device address
 * @param regAddr Register regAddr to read from
 * @param data Container for word value read from device
 * @param timeout Optional read timeout in milliseconds (0 to disable, leave off to use default class value in I2C::readTimeout)
 * @return Status of read operation (true = success)
 */
int8_t I2C::readWord(uint8_t devAddr, uint8_t regAddr, uint16_t *data, uint16_t timeout) {
    return readWords(devAddr, regAddr, 1, data, timeout);
}

/** Read multiple bytes from an 8-bit device register.
 * @param devAddr I2C slave device address
 * @param regAddr First register regAddr to read from
 * @param length Number of bytes to read
 * @param data Buffer to store read data in
 * @param timeout Optional read timeout in milliseconds (0 to disable, leave off to use default class value in I2C::readTimeout)
 * @return Number of bytes read (-1 indicates failure)
 */
int8_t I2C::readBytes(uint8_t devAddr, uint8_t regAddr, uint8_t length, uint8_t *data, uint16_t timeout) {
    int8_t count = 0;
    //lock mutex
    pthread_mutex_lock(&I2C::I2C_Mutex);

    int fd = open("/dev/i2c-3", O_RDWR);

    if (fd < 0) {
        fprintf(stderr, "Failed to open device: %s\n", strerror(errno));
        return(-1);
    }
    if (ioctl(fd, I2C_SLAVE, devAddr) < 0) {
        fprintf(stderr, "Failed to select device: %s\n", strerror(errno));
        close(fd);
        return(-1);
    }
    if (write(fd, &regAddr, 1) != 1) {
        fprintf(stderr, "Failed to write reg: %s\n", strerror(errno));
        close(fd);
        return(-1);
    }
    count = read(fd, data, length);
    if (count < 0) {
        fprintf(stderr, "Failed to read device(%d): %s\n", count, ::strerror(errno));
        close(fd);
        return(-1);
    } else if (count != length) {
        fprintf(stderr, "Short read  from device, expected %d, got %d\n", length, count);
        close(fd);
        return(-1);
    }
    close(fd);

    //unlock mutex
    pthread_mutex_unlock(&I2C::I2C_Mutex);

    return count;
}

/** Read multiple words from a 16-bit device register.
 * @param devAddr I2C slave device address
 * @param regAddr First register regAddr to read from
 * @param length Number of words to read
 * @param data Buffer to store read data in
 * @param timeout Optional read timeout in milliseconds (0 to disable, leave off to use default class value in I2C::readTimeout)
 * @return Number of words read (0 indicates failure)
 */
int8_t I2C::readWords(uint8_t devAddr, uint8_t regAddr, uint8_t length, uint16_t *data, uint16_t timeout) {
    int8_t count = 0;

    printf("ReadWords() not implemented\n");
    // Use readBytes() and potential byteswap
    *data = 0; // keep the compiler quiet

    return count;
}

/** write a single bit in an 8-bit device register.
 * @param devAddr I2C slave device address
 * @param regAddr Register regAddr to write to
 * @param bitNum Bit position to write (0-7)
 * @param value New bit value to write
 * @return Status of operation (true = success)
 */
bool I2C::writeBit(uint8_t devAddr, uint8_t regAddr, uint8_t bitNum, uint8_t data) {
    uint8_t b;
    readByte(devAddr, regAddr, &b);
    b = (data != 0) ? (b | (1 << bitNum)) : (b & ~(1 << bitNum));
    return writeByte(devAddr, regAddr, b);
}

/** write a single bit in a 16-bit device register.
 * @param devAddr I2C slave device address
 * @param regAddr Register regAddr to write to
 * @param bitNum Bit position to write (0-15)
 * @param value New bit value to write
 * @return Status of operation (true = success)
 */
bool I2C::writeBitW(uint8_t devAddr, uint8_t regAddr, uint8_t bitNum, uint16_t data) {
    uint16_t w;
    readWord(devAddr, regAddr, &w);
    w = (data != 0) ? (w | (1 << bitNum)) : (w & ~(1 << bitNum));
    return writeWord(devAddr, regAddr, w);
}

/** Write multiple bits in an 8-bit device register.
 * @param devAddr I2C slave device address
 * @param regAddr Register regAddr to write to
 * @param bitStart First bit position to write (0-7)
 * @param length Number of bits to write (not more than 8)
 * @param data Right-aligned value to write
 * @return Status of operation (true = success)
 */
bool I2C::writeBits(uint8_t devAddr, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t data) {
    //      010 value to write
    // 76543210 bit numbers
    //    xxx   args: bitStart=4, length=3
    // 00011100 mask byte
    // 10101111 original value (sample)
    // 10100011 original & ~mask
    // 10101011 masked | value
    uint8_t b;
    if (readByte(devAddr, regAddr, &b) != 0) {
        uint8_t mask = ((1 << length) - 1) << (bitStart - length + 1);
        data <<= (bitStart - length + 1); // shift data into correct position
        data &= mask; // zero all non-important bits in data
        b &= ~(mask); // zero all important bits in existing byte
        b |= data; // combine data with existing byte
        return writeByte(devAddr, regAddr, b);
    } else {
        return false;
    }
}

/** Write multiple bits in a 16-bit device register.
 * @param devAddr I2C slave device address
 * @param regAddr Register regAddr to write to
 * @param bitStart First bit position to write (0-15)
 * @param length Number of bits to write (not more than 16)
 * @param data Right-aligned value to write
 * @return Status of operation (true = success)
 */
bool I2C::writeBitsW(uint8_t devAddr, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint16_t data) {
    //              010 value to write
    // fedcba9876543210 bit numbers
    //    xxx           args: bitStart=12, length=3
    // 0001110000000000 mask byte
    // 1010111110010110 original value (sample)
    // 1010001110010110 original & ~mask
    // 1010101110010110 masked | value
    uint16_t w;
    if (readWord(devAddr, regAddr, &w) != 0) {
        uint8_t mask = ((1 << length) - 1) << (bitStart - length + 1);
        data <<= (bitStart - length + 1); // shift data into correct position
        data &= mask; // zero all non-important bits in data
        w &= ~(mask); // zero all important bits in existing word
        w |= data; // combine data with existing word
        return writeWord(devAddr, regAddr, w);
    } else {
        return false;
    }
}

/** Write single byte to an 8-bit device register.
 * @param devAddr I2C slave device address
 * @param regAddr Register address to write to
 * @param data New byte value to write
 * @return Status of operation (true = success)
 */
bool I2C::writeByte(uint8_t devAddr, uint8_t regAddr, uint8_t data) {
    return writeBytes(devAddr, regAddr, 1, &data);
}

/** Write single word to a 16-bit device register.
 * @param devAddr I2C slave device address
 * @param regAddr Register address to write to
 * @param data New word value to write
 * @return Status of operation (true = success)
 */
bool I2C::writeWord(uint8_t devAddr, uint8_t regAddr, uint16_t data) {
    return writeWords(devAddr, regAddr, 1, &data);
}

/** Write multiple bytes to an 8-bit device register.
 * @param devAddr I2C slave device address
 * @param regAddr First register address to write to
 * @param length Number of bytes to write
 * @param data Buffer to copy new data from
 * @return Status of operation (true = success)
 */
bool I2C::writeBytes(uint8_t devAddr, uint8_t regAddr, uint8_t length, uint8_t* data) {
    int8_t count = 0;
    uint8_t buf[128];
    int fd;

    if (length > 127) {
        fprintf(stderr, "Byte write count (%d) > 127\n", length);
        return(FALSE);
    }
    //lock mutex
    pthread_mutex_lock(&I2C::I2C_Mutex);


    fd = open("/dev/i2c-3", O_RDWR);
    if (fd < 0) {
        fprintf(stderr, "Failed to open device: %s\n", strerror(errno));
        return(FALSE);
    }
    if (ioctl(fd, I2C_SLAVE, devAddr) < 0) {
        fprintf(stderr, "Failed to select device: %s\n", strerror(errno));
        close(fd);
        return(FALSE);
    }
    buf[0] = regAddr;
    memcpy(buf+1,data,length);
    count = write(fd, buf, length+1);
    if (count < 0) {
        fprintf(stderr, "Failed to write device(%d): %s\n", count, ::strerror(errno));
        close(fd);
        return(FALSE);
    } else if (count != length+1) {
        fprintf(stderr, "Short write to device, expected %d, got %d\n", length+1, count);
        close(fd);
        return(FALSE);
    }
    close(fd);

    //unlock mutex
    pthread_mutex_unlock(&I2C::I2C_Mutex);

    return TRUE;
}

/** Write multiple words to a 16-bit device register.
 * @param devAddr I2C slave device address
 * @param regAddr First register address to write to
 * @param length Number of words to write
 * @param data Buffer to copy new data from
 * @return Status of operation (true = success)
 */
bool I2C::writeWords(uint8_t devAddr, uint8_t regAddr, uint8_t length, uint16_t* data) {
    int8_t count = 0;
    uint8_t buf[128];
    int i, fd;

    // Should do potential byteswap and call writeBytes() really, but that
    // messes with the callers buffer

    if (length > 63) {
        fprintf(stderr, "Word write count (%d) > 63\n", length);
        return(FALSE);
    }

    //lock mutex
    pthread_mutex_lock(&I2C::I2C_Mutex);

    fd = open("/dev/i2c-3", O_RDWR);
    if (fd < 0) {
        fprintf(stderr, "Failed to open device: %s\n", strerror(errno));
        return(FALSE);
    }
    if (ioctl(fd, I2C_SLAVE, devAddr) < 0) {
        fprintf(stderr, "Failed to select device: %s\n", strerror(errno));
        close(fd);
        return(FALSE);
    }
    buf[0] = regAddr;
    for (i = 0; i < length; i++) {
        buf[i*2+1] = data[i] >> 8;
        buf[i*2+2] = data[i];
    }
    count = write(fd, buf, length*2+1);
    if (count < 0) {
        fprintf(stderr, "Failed to write device(%d): %s\n", count, ::strerror(errno));
        close(fd);
        return(FALSE);
    } else if (count != length*2+1) {
        fprintf(stderr, "Short write to device, expected %d, got %d\n", length+1, count);
        close(fd);
        return(FALSE);
    }
    close(fd);

    //unlock mutex
    pthread_mutex_unlock(&I2C::I2C_Mutex);

    return TRUE;
}

/** Default timeout value for read operations.
 * Set this to 0 to disable timeout detection.
 */
uint16_t I2C::readTimeout = 0;
pthread_mutex_t I2C::I2C_Mutex;

