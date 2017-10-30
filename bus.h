#ifndef BUS_h
#define BUS_h

#include "Arduino.h"

class Bus {
public:
    virtual void begin(){};

    virtual void end(){};

    virtual uint8_t readByte(uint8_t address, uint8_t subAddress, bool fast = false) = 0;

    virtual bool writeByte(uint8_t address, uint8_t subAddress, uint8_t data) = 0;

    virtual void readBytes(uint8_t address, uint8_t subAddress, uint8_t count, uint8_t* dest, bool fast = false) = 0;
};

#endif