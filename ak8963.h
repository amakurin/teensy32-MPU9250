#ifndef AK8963_h
#define AK8963_h


#include "Arduino.h"
#include "bus.h"
#include "utils.h"

class AK8963 {
public:
    static const uint8_t I2C_ADDRESS  =  0x0C;
    // Registers
    static const uint8_t WIA      =  0x00;  // R  Device Id, should return 0x48
    static const uint8_t INFO     =  0x01;  // R  
    static const uint8_t ST1      =  0x02;  // R  data ready status bit 0
    static const uint8_t HXL      =  0x03;  // R  data
    static const uint8_t HXH      =  0x04;  // R  
    static const uint8_t HYL      =  0x05;  // R  
    static const uint8_t HYH      =  0x06;  // R  
    static const uint8_t HZL      =  0x07;  // R  
    static const uint8_t HZH      =  0x08;  // R  
    static const uint8_t ST2      =  0x09;  // R  Data overflow bit 3 and data read error status bit 2
    static const uint8_t CNTL1    =  0x0A;  // RW Control (see control commands)
    static const uint8_t CNTL2    =  0x0B;
    static const uint8_t ASTC     =  0x0C;  // RW Self test control
    static const uint8_t I2CDIS   =  0x0F;  // RW I2C disable
    static const uint8_t ASAX     =  0x10;  // R  Fuse ROM x-axis sensitivity adjustment value
    static const uint8_t ASAY     =  0x11;  // R  Fuse ROM y-axis sensitivity adjustment value
    static const uint8_t ASAZ     =  0x12;  // R  Fuse ROM z-axis sensitivity adjustment value
    // CNTL1 commands // MODE[3:0] + BIT[4]: 1 (0) to enable 16 (14) bit resolution
    static const uint8_t CNTL1_PWR_DOWN     =  0x00;  // "0000": Power-down mode
    static const uint8_t CNTL1_SINGLE_MEAS  =  0x01;  // "0001": Single measurement mode
    static const uint8_t CNTL1_CONT_MEAS_1  =  0x12;  // "0010": Continuous measurement mode 1 8Hz
    static const uint8_t CNTL1_CONT_MEAS_2  =  0x16;  // "0110": Continuous measurement mode 2 100Hz
    static const uint8_t CNTL1_EXT_TRIG     =  0x04;  // "0100": External trigger measurement mode
    static const uint8_t CNTL1_SELF_TEST    =  0x08;  // "1000": Self-test mode
    static const uint8_t CNTL1_FUSE         =  0x0F;  // "1111": Fuse ROM access mode
    // CNTL2 commands 
    static const uint8_t CNTL2_RESET        =  0x01;

    Bus* _bus;

    float _magCalibration[3] = {0, 0, 0};  // Factory mag calibration
    float _magBias[3] = {12.5, 15.0,  -33.0};  // Calibration bias
    float _magScale[3] = {1.02,  1.04,   0.95};  // Calibration scale

    AK8963 (Bus* bus){
        this->_bus = bus;
    }

    void setup()
    {
        powerDown();  
        fuseAccess();

        // Extract the factory calibration for each magnetometer axis
        uint8_t rawData[3];  
        readRegisters(ASAX, 3, &rawData[0]);  // Read the x-, y-, and z-axis calibration values
        float magnetometerResolution = 4912.0f / 32760.0f; // micro Tesla
        _magCalibration[0] = ((float)(rawData[0] - 128)/256. + 1.) * magnetometerResolution; 
        _magCalibration[1] = ((float)(rawData[1] - 128)/256. + 1.) * magnetometerResolution;  
        _magCalibration[2] = ((float)(rawData[2] - 128)/256. + 1.) * magnetometerResolution; 

        powerDown();  
        writeRegister(CNTL1, CNTL1_CONT_MEAS_2); // Set mode to 16 bit resolution at 100Hz 
    }

    bool readDataRaw(int16_t* dest){
        uint8_t buffer[7];
        readRegisters(HXL, 7, buffer);  
        to16bit(&buffer[0], dest, 3, true);
        return !isOverflow(buffer[6]);
    }

    void calibrate(float* bias, float* scale){
        uint16_t ii = 0, sample_count = 1500; // at 100 Hz ODR, new mag data is available every 10 ms
        int32_t int32_bias[3] = {0, 0, 0}, int32_scale[3] = {0, 0, 0};
        int16_t mag_max[3] = {-32767, -32767, -32767}, mag_min[3] = {32767, 32767, 32767}, mag_temp[3] = {0, 0, 0};
        delay(4000);

        // shoot for ~fifteen seconds of mag data
        for(ii = 0; ii < sample_count; ii++) {
            if (readDataRaw(mag_temp)){  // Read the mag data   
                for (int jj = 0; jj < 3; jj++) {
                    if(mag_temp[jj] > mag_max[jj]) mag_max[jj] = mag_temp[jj];
                    if(mag_temp[jj] < mag_min[jj]) mag_min[jj] = mag_temp[jj];
                }
            }
            delay(12);  // at 100 Hz ODR, new mag data is available every 10 ms
        }

        // Get hard iron correction
        int32_bias[0]  = (mag_max[0] + mag_min[0])/2;  // get average x mag bias in counts
        int32_bias[1]  = (mag_max[1] + mag_min[1])/2;  // get average y mag bias in counts
        int32_bias[2]  = (mag_max[2] + mag_min[2])/2;  // get average z mag bias in counts

        bias[0] = (float) int32_bias[0] * _magCalibration[0];  // save mag biases in uT for main program
        bias[1] = (float) int32_bias[1] * _magCalibration[1];   
        bias[2] = (float) int32_bias[2] * _magCalibration[2];  

        // Get soft iron correction estimate
        int32_scale[0]  = (mag_max[0] - mag_min[0])/2;  // get average x axis max chord length in counts
        int32_scale[1]  = (mag_max[1] - mag_min[1])/2;  // get average y axis max chord length in counts
        int32_scale[2]  = (mag_max[2] - mag_min[2])/2;  // get average z axis max chord length in counts

        float avg_rad = int32_scale[0] + int32_scale[1] + int32_scale[2];
        avg_rad /= 3.0;

        scale[0] = avg_rad/((float)int32_scale[0]);
        scale[1] = avg_rad/((float)int32_scale[1]);
        scale[2] = avg_rad/((float)int32_scale[2]);
    }

    bool isOverflow(uint8_t value){
        return (value & 0x08);
    }

    byte WhoAmI(){
        return _bus->readByte(I2C_ADDRESS, WIA);  
    }

    void powerDown(){
        writeRegister(CNTL1, CNTL1_PWR_DOWN); // Power down magnetometer  
    }

    void reset(){
        writeRegister(CNTL2, CNTL2_RESET);
    }

    void fuseAccess(){
        writeRegister(CNTL1, CNTL1_FUSE); // Enter Fuse ROM access mode
    }

    void writeRegister(uint8_t address, uint8_t data){
        _bus->writeByte(I2C_ADDRESS, address, data);
    }

    void readRegisters(uint8_t address, uint8_t count, uint8_t* dest){
        _bus->readBytes(I2C_ADDRESS, address, count, dest);
    }

};

#endif