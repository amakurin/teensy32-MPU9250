#ifndef I2CBus_h
#define I2CBus_h

#include "Arduino.h"
#include "i2c_t3.h"  // I2C library
#include "bus.h"

class I2CBus : public Bus {
public:
    static const uint8_t I2C_SCL_PIN = 19;
    static const uint8_t I2C_SDA_PIN = 18;
    static const uint8_t TEENSY_I2C_BUS = 0;
    static const uint32_t I2C_RATE = 400000;
    i2c_t3 _i2c;

    I2CBus() : _i2c(TEENSY_I2C_BUS) {
    };

    void begin(){
        // address is zero as ignored by master mode
        _i2c.begin(I2C_MASTER, 0, I2C_PINS_18_19, I2C_PULLUP_EXT, I2C_RATE); 
    }

    void end(){
        pinMode(I2C_SCL_PIN, INPUT);
        digitalWrite(I2C_SCL_PIN, LOW);
        pinMode(I2C_SDA_PIN, INPUT);
        digitalWrite(I2C_SDA_PIN, LOW);
    }
    
    uint8_t readByte(uint8_t address, uint8_t subAddress, bool fast = false)
    {
        uint8_t data;      
        _i2c.beginTransmission(address);  
        _i2c.write(subAddress);           
        _i2c.endTransmission(I2C_NOSTOP); 
        _i2c.requestFrom(address, (size_t) 1);     
        data = _i2c.read();               
        return data;                      
    }

    bool writeByte(uint8_t address, uint8_t subAddress, uint8_t data)
    {
        _i2c.beginTransmission(address);  
        _i2c.write(subAddress);           
        _i2c.write(data);                 
        _i2c.endTransmission();           
        
        delay(10); 

        // check if register was updated
        uint8_t actual_val = readByte(address, subAddress);
        return (actual_val == data);
    }

    void readBytes(uint8_t address, uint8_t subAddress, uint8_t count, uint8_t* dest, bool fast = false){
        _i2c.beginTransmission(address); // open the device
        _i2c.write(subAddress); // specify the starting register address
        _i2c.endTransmission(I2C_NOSTOP);
        _i2c.requestFrom(address, count); // specify the number of bytes to receive
        uint8_t i = 0; // read the data into the buffer
        while( _i2c.available() ){
            dest[i++] = _i2c.readByte();
        }
    };
};

#endif