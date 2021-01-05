/*
MS5611-01BA.h - Interfaces a Measurement Specialities MS5611-01BA with Arduino
See http://www.meas-spec.com/downloads/MS5611-01BA01.pdf for the device datasheet

Copyright (C) 2011 Fabio Varesano <fvaresano@yahoo.it>

Development of this code has been supported by the Department of Computer Science, Universita' degli Studi di Torino, Italy within the Piemonte Project http://www.piemonte.di.unito.it/


This program is free software: you can redistribute it and/or modify
it under the terms of the version 3 GNU General Public License as
published by the Free Software Foundation.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.

*/


#ifndef MS561101BA_h
#define MS561101BA_h

#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif

#include "Arduino.h"
#include <Wire.h>

//#define DEBUG_V
//#define DEBUG
//#include <DebugUtils.h>

// addresses of the device
#define MS561101BA_ADDR_CSB_HIGH  0x76   //CBR=1 0x76 I2C address when CSB is connected to HIGH (VCC)
#define MS561101BA_ADDR_CSB_LOW   0x77   //CBR=0 0x77 I2C address when CSB is connected to LOW (GND)

// registers of the device
#define MS561101BA_D1 0x40
#define MS561101BA_D2 0x50
#define MS561101BA_RESET 0x1E

// D1 and D2 result size (bytes)
#define MS561101BA_D1D2_SIZE 3

// OSR (Over Sampling Ratio) constants
#define MS561101BA_OSR_256 0x00
#define MS561101BA_OSR_512 0x02
#define MS561101BA_OSR_1024 0x04
#define MS561101BA_OSR_2048 0x06
#define MS561101BA_OSR_4096 0x08

#define MS561101BA_PROM_BASE_ADDR 0xA2 // by adding ints from 0 to 6 we can read all the prom configuration values. 
// C1 will be at 0xA2 and all the subsequent are multiples of 2
#define MS561101BA_PROM_REG_COUNT 6 // number of registers in the PROM
#define MS561101BA_PROM_REG_SIZE 2 // size in bytes of a prom registry.



class MS561101BA {
  public:
    MS561101BA();
    void init(uint8_t addr);
    float getPressure(uint8_t OSR);
    float getTemperature(uint8_t OSR);
    int32_t getDeltaTemp(uint8_t OSR);
    uint32_t rawPressure(uint8_t OSR);
    uint32_t rawTemperature(uint8_t OSR);
    int readPROM();
    void reset();
    uint32_t lastPresConv, lastTempConv;
  private:
    void startConversion(uint8_t command);
    uint32_t getConversion(uint8_t command);
    uint8_t _addr;
    uint16_t _C[MS561101BA_PROM_REG_COUNT];
    uint32_t pressCache, tempCache;
};

#endif // MS561101BA_h
#define CONVERSION_TIME 10000l // conversion time in microseconds

/*
void printLongLong(uint64_t n, uint8_t base) {
  do {
    uint64_t m = n;
    n /= base;
    char c = m - base * n;
    c = c < 10 ? c + '0' : c + 'A' - 10;
    Serial.print(c);
  } while(n);
  Serial.println();
}
*/


MS561101BA::MS561101BA() {
  ;
}

void MS561101BA::init(uint8_t address) {  
  _addr =  address;
  
  // disable internal pullups of the ATMEGA which Wire enable by default
  #if defined(__AVR_ATmega168__) || defined(__AVR_ATmega8__) || defined(__AVR_ATmega328P__)
    // deactivate internal pull-ups for twi
    // as per note from atmega8 manual pg167
    cbi(PORTC, 4);
    cbi(PORTC, 5);
  #else
    // deactivate internal pull-ups for twi
    // as per note from atmega128 manual pg204
    cbi(PORTD, 0);
    cbi(PORTD, 1);
  #endif
 
  reset(); // reset the device to populate its internal PROM registers
  delay(1000); // some safety time
  readPROM(); // reads the PROM into object variables for later use
}

float MS561101BA::getPressure(uint8_t OSR) {
  // see datasheet page 7 for formulas
  
  uint32_t rawPress = rawPressure(OSR);
  if(rawPress == NULL) {
    return NULL;
  }
  
  int32_t dT = getDeltaTemp(OSR);
  if(dT == NULL) {
    return NULL;
  }
  
  int64_t off  = ((uint32_t)_C[1] <<16) + (((int64_t)dT * _C[3]) >> 7);
  int64_t sens = ((uint32_t)_C[0] <<15) + (((int64_t)dT * _C[2]) >> 8);
  return ((( (rawPress * sens ) >> 21) - off) >> 15) / 100.0;
}

float MS561101BA::getTemperature(uint8_t OSR) {
  // see datasheet page 7 for formulas
  int64_t dT = getDeltaTemp(OSR);
  
  if(dT != NULL) {
    return (2000 + ((dT * _C[5]) >> 23)) / 100.0;
  }
  else {
    return NULL;
  }
}

int32_t MS561101BA::getDeltaTemp(uint8_t OSR) {
  uint32_t rawTemp = rawTemperature(OSR);
  if(rawTemp != NULL) {
    return (int32_t)(rawTemp - ((uint32_t)_C[4] << 8));
  }
  else {
    return NULL;
  }
}

//TODO: avoid duplicated code between rawPressure and rawTemperature methods
//TODO: possible race condition between readings.. serious headache doing this.. help appreciated!

uint32_t MS561101BA::rawPressure(uint8_t OSR) {
  unsigned long now = micros();
  if(lastPresConv != 0 && (now - lastPresConv) >= CONVERSION_TIME) {
    lastPresConv = 0;
    pressCache = getConversion(MS561101BA_D1 + OSR);
  }
  else {
    if(lastPresConv == 0 && lastTempConv == 0) {
      startConversion(MS561101BA_D1 + OSR);
      lastPresConv = now;
    }
    return pressCache;
  }
}

uint32_t MS561101BA::rawTemperature(uint8_t OSR) {
  unsigned long now = micros();
  if(lastTempConv != 0 && (now - lastTempConv) >= CONVERSION_TIME) {
    lastTempConv = 0;
    tempCache = getConversion(MS561101BA_D2 + OSR);
  }
  else {
    if(lastTempConv == 0 && lastPresConv == 0) { // no conversions in progress
      startConversion(MS561101BA_D2 + OSR);
      lastTempConv = now;
    }
  }
  return tempCache;
}


// see page 11 of the datasheet
void MS561101BA::startConversion(uint8_t command) {
  // initialize pressure conversion
  Wire.beginTransmission(_addr);
  Wire.write(command);
  Wire.endTransmission();
}

uint32_t MS561101BA::getConversion(uint8_t command) {
  union {uint32_t val; uint8_t raw[4]; } conversion = {0};
  
  // start read sequence
  Wire.beginTransmission(_addr);
  Wire.write(0);
  Wire.endTransmission();
  
  Wire.beginTransmission(_addr);
  Wire.requestFrom(_addr, (uint8_t) MS561101BA_D1D2_SIZE);
  if(Wire.available()) {
    conversion.raw[2] = Wire.read();
    conversion.raw[1] = Wire.read();
    conversion.raw[0] = Wire.read();
  }
  else {
    conversion.val = -1;
  }
  
  return conversion.val;
}


/**
 * Reads factory calibration and store it into object variables.
*/
int MS561101BA::readPROM() {
  for (int i=0;i<MS561101BA_PROM_REG_COUNT;i++) {
    Wire.beginTransmission(_addr);
    Wire.write(MS561101BA_PROM_BASE_ADDR + (i * MS561101BA_PROM_REG_SIZE));
    Wire.endTransmission();
    
    Wire.beginTransmission(_addr);
    Wire.requestFrom(_addr, (uint8_t) MS561101BA_PROM_REG_SIZE);
    if(Wire.available()) {
      _C[i] = Wire.read() << 8 | Wire.read();
      
      //DEBUG_PRINT(_C[i]);
    }
    else {
      return -1; // error reading the PROM or communicating with the device
    }
  }
  return 0;
}


/**
 * Send a reset command to the device. With the reset command the device
 * populates its internal registers with the values read from the PROM.
*/
void MS561101BA::reset() {
  Wire.beginTransmission(_addr);
  Wire.write(MS561101BA_RESET);
  Wire.endTransmission();
}

