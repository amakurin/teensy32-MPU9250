#ifndef SPIBus_h
#define SPIBus_h

#include "Arduino.h"
#include "SPI.h"  // I2C library
#include "bus.h"

class SPIBus : public Bus {
public:
    static const uint8_t SPI_CS_PIN = 15;
    static const uint8_t SPI_CLCK_PIN = 13;
    static const uint8_t SPI_DIN_PIN = 12;
    static const uint8_t SPI_DOUT_PIN = 11;
    static const uint8_t SPI_READ = 0x80;
    static const uint32_t SPI_LS_CLOCK = 1000000; // 1 MHz
    static const uint32_t SPI_HS_CLOCK = 20000000; // 20 MHz

    SPIClass* _spi;
    uint8_t _csPin;
    uint8_t _clckPin;
    uint8_t _dinPin;
    uint8_t _doutPin;

    SPIBus() : SPIBus(SPI_CS_PIN, SPI_CLCK_PIN, SPI_DIN_PIN, SPI_DOUT_PIN) {
    };

    SPIBus(uint8_t csPin, uint8_t clckPin, uint8_t dinPin, uint8_t doutPin) : _spi(&SPI), 
        _csPin(csPin), _clckPin(clckPin), _dinPin(dinPin), _doutPin(doutPin) {
        pinMode(_csPin, INPUT);
        digitalWrite(_csPin, LOW);
        pinMode(_clckPin, INPUT);
        digitalWrite(_clckPin, LOW);
        pinMode(_dinPin, INPUT);
        digitalWrite(_dinPin, LOW);
        pinMode(_doutPin, INPUT);
        digitalWrite(_doutPin, LOW);
    };

    void begin(){
        pinMode(_csPin, OUTPUT);
        digitalWriteFast(_csPin, HIGH);
        _spi->begin();
    }


    void end(){
        _spi->end();
        pinMode(_csPin, INPUT);
        digitalWrite(_csPin, LOW);
        pinMode(_clckPin, INPUT);
        digitalWrite(_clckPin, LOW);
        pinMode(_dinPin, INPUT);
        digitalWrite(_dinPin, LOW);
        pinMode(_doutPin, INPUT);
        digitalWrite(_doutPin, LOW);
    }

    uint8_t readByte(uint8_t address, uint8_t subAddress, bool fast = false)
    {
        uint8_t data;      
        readBytes(address, subAddress, 1, &data, fast);
        return data;                      
    }

    bool writeByte(uint8_t address, uint8_t subAddress, uint8_t data)
    {
        _spi->beginTransaction(SPISettings(SPI_LS_CLOCK, MSBFIRST, SPI_MODE3)); // begin the transaction
        digitalWriteFast(_csPin,LOW); // select the MPU9250 chip
        _spi->transfer(subAddress); // write the register address
        _spi->transfer(data); // write the data
        digitalWriteFast(_csPin,HIGH); // deselect the MPU9250 chip
        _spi->endTransaction(); // end the transaction
        return true;
    }

    void readBytes(uint8_t address, uint8_t subAddress, uint8_t count, uint8_t* dest, bool fast = false){
        _spi->beginTransaction(SPISettings(fast ? SPI_HS_CLOCK : SPI_LS_CLOCK, MSBFIRST, SPI_MODE3));   
        digitalWriteFast(_csPin,LOW); // select the MPU9250 chip

        _spi->transfer(subAddress | SPI_READ); // specify the starting register address

        digitalWriteFast(_csPin,HIGH); // deselect the MPU9250 chip
        delayMicroseconds(1);
        digitalWriteFast(_csPin,LOW); // select the MPU9250 chip

        _spi->transfer(subAddress | SPI_READ); // specify the starting register address

        for(uint8_t i = 0; i < count; i++){
            dest[i] = _spi->transfer(0x00); // read the data
        }

        digitalWriteFast(_csPin,HIGH); // deselect the MPU9250 chip
        _spi->endTransaction(); // end the transaction
    };
};

#endif