#ifndef MPU9250_h
#define MPU9250_h

#include "Arduino.h"
#include "bus.h"
#include "ak8963.h"
#include "utils.h"

class MPU9250 {
public:
    static const uint8_t MPU9250_I2C_ADDRESS = 0x68;

    static const uint8_t CLOCK_SEL_PLL  = 0x01;
    static const uint8_t XG_OFFSET_H    = 0x13;  // User-defined trim values for gyroscope
    static const uint8_t XG_OFFSET_L    = 0x14;
    static const uint8_t YG_OFFSET_H    = 0x15;
    static const uint8_t YG_OFFSET_L    = 0x16;
    static const uint8_t ZG_OFFSET_H    = 0x17;
    static const uint8_t ZG_OFFSET_L    = 0x18;
    static const uint8_t SMPLRT_DIV     = 0x19;
    static const uint8_t CONFIG         = 0x1A;
    static const uint8_t GYRO_CONFIG    = 0x1B;
    static const uint8_t ACCEL_CONFIG   = 0x1C;
    static const uint8_t ACCEL_CONFIG2  = 0x1D;
    static const uint8_t LP_ACCEL_ODR   = 0x1E;   
    static const uint8_t WOM_THR        = 0x1F;   
    static const uint8_t MOT_DUR        = 0x20;  // Duration counter threshold for motion interrupt generation, 1 kHz rate, LSB = 1 ms
    static const uint8_t ZMOT_THR       = 0x21;  // Zero-motion detection threshold bits [7:0]
    static const uint8_t ZRMOT_DUR      = 0x22;  // Duration counter threshold for zero motion interrupt generation, 16 Hz rate, LSB = 64 ms
    static const uint8_t FIFO_EN        = 0x23;
    static const uint8_t I2C_MST_CTRL   = 0x24; 
    static const uint8_t I2C_MST_CLK    = 0x0D;  // BIT[3:0]= 0x0D - 400 kHz I2C
    static const uint8_t WAIT_FOR_ES    = 6;     // BIT[6] Delays the data ready interrupt until external sensor data is loaded.
    static const uint8_t I2C_MST_P_NSR  = 4;     // This bit controls the I2C Master’s transition from one slave read to the next slave read. If 0, there is a restart between reads. If 1, there is a stop between reads.

    static const uint8_t I2C_SLV0_ADDR  = 0x25;
    static const uint8_t I2C_SLV0_REG   = 0x26;
    static const uint8_t I2C_SLV0_CTRL  = 0x27;
    static const uint8_t I2C_SLV0_EN    = 0x80;

    static const uint8_t I2C_READ_FLAG  = 0x80;
    static const uint8_t I2C_READ_COUNT = 0x07;  // read 7 bytes from mag xl, xh, yl, yh, zl, zh, st2 - status 
   
    static const uint8_t INT_PIN_CFG    = 0x37;
    static const uint8_t BYPASS_EN      = 1; 

    static const uint8_t INT_ENABLE     = 0x38;
    static const uint8_t RAW_RDY_EN     = 0;     // BIT[0] Enable Raw Sensor Data Ready interrupt to propagate to interrupt pin.

    static const uint8_t ACCEL_OUT      = 0x3B;

    static const uint8_t USER_CTRL      = 0x6A; 
    static const uint8_t I2C_IF_DIS     = 4;
    static const uint8_t I2C_MST_EN     = 5;

    static const uint8_t PWR_MGMT_1     = 0x6B;
    static const uint8_t H_RESET        = 7;
    static const uint8_t PWR_MGMT_2     = 0x6C;

    static const uint8_t FIFO_COUNTH    = 0x72;
    static const uint8_t FIFO_R_W       = 0x74;
    static const uint8_t XA_OFFSET_H    = 0x77;
    static const uint8_t XA_OFFSET_L    = 0x78;
    static const uint8_t YA_OFFSET_H    = 0x7A;
    static const uint8_t YA_OFFSET_L    = 0x7B;
    static const uint8_t ZA_OFFSET_H    = 0x7D;
    static const uint8_t ZA_OFFSET_L    = 0x7E;

    static constexpr float G = 9.807f;
    static constexpr float accelScale = G * 2.0f/32767.5f;
    static constexpr float d2r = 3.14159265359f/180.0f;
    static constexpr float gyroScale = 250.0f/32767.5f * d2r;
    static constexpr float tempScale = 333.87f;
    static constexpr float tempOffset = 21.0f;


    Bus* _i2cbus;
    Bus* _spibus;
    Bus* _bus;
    AK8963 _mag;
    MPU9250 (Bus* i2cbus, Bus* spibus) : _i2cbus(i2cbus),  _spibus(spibus), _bus(_i2cbus), _mag(_i2cbus) {
        _i2cbus->begin();
    }

    void switchMasterBus(bool useSPI = false){
        if (useSPI){
            writeRegisterBit(USER_CTRL, I2C_IF_DIS); 
            _bus->end();
            _bus = _spibus; 
        }
        else{
            writeRegisterBit(USER_CTRL, I2C_IF_DIS, 0); 
            _bus->end();
            _bus = _i2cbus; 
        }
        _bus->begin();
    }

    void toBypassMode() {
        writeRegisterBit(PWR_MGMT_1, H_RESET);      // Power reset
        writeRegister(INT_PIN_CFG, 0x02);           // Enable I2C bypass, disable FSYNC
        writeRegisterBit(USER_CTRL, I2C_MST_EN, 0); // Disable I2C Master Mode
    }

// Enabling interrupts with connecting INT to teensy pin causes magnetometer overflow and freezing on my mpu chip
//    void enableInterrupts(){
//        writeRegisterBit(INT_PIN_CFG, 4);           // clear interrupt on any read
//        writeRegisterBit(INT_PIN_CFG, 5);           
//        writeRegisterBit(I2C_MST_CTRL, WAIT_FOR_ES);   // WAIT_FOR_ES
//        writeRegisterBit(INT_ENABLE, 0);    // RAW_RDY_EN
//    }

//    void disableInterrupts(){
//        writeRegisterBit(I2C_MST_CTRL, WAIT_FOR_ES, 0);   
//        writeRegisterBit(INT_ENABLE, 0, 0);   
//    }

    void setup() {
        toBypassMode();
        _mag.setup();
        delay(100);
        writeRegisterBit(PWR_MGMT_1, H_RESET);
        float  gyroBias[3], accelBias[3];
        calibrate(&gyroBias[0], &accelBias[0]);

        Serial.print("Gyro Bias: [\t");
        Serial.print(gyroBias[0]);
        Serial.print(",\t");
        Serial.print(gyroBias[1]);
        Serial.print(",\t");
        Serial.print(gyroBias[2]);
        Serial.println("]");
        Serial.print("Accel Bias: [\t");
        Serial.print(accelBias[0]);
        Serial.print(",\t");
        Serial.print(accelBias[1]);
        Serial.print(",\t");
        Serial.print(accelBias[2]);
        Serial.println("]");

        writeRegisterBit(PWR_MGMT_1, H_RESET);

        writeRegister(I2C_SLV0_ADDR, I2C_READ_FLAG | AK8963::I2C_ADDRESS); // Set slave_0 to the AK8963 and read mode
        delay(10);
        writeRegister(I2C_SLV0_REG, AK8963::HXL);   // Set slave_0 read register to AK8963 HXL
        delay(10);
        writeRegister(I2C_SLV0_CTRL, I2C_SLV0_EN | I2C_READ_COUNT); // enable I2C and request the bytes
        delay(10);
        writeRegister(I2C_MST_CTRL, I2C_MST_CLK | (1 << I2C_MST_P_NSR));   // set i2c to 400Hz
        delay(10);
        writeRegisterBit(USER_CTRL, I2C_MST_EN);    // Enable I2C Master Mode
        delay(10);
    }

    // Function which accumulates gyro and accelerometer data after device initialization. It calculates the average
    // of the at-rest readings and then loads the resulting offsets into accelerometer and gyro bias registers.
    void calibrate(float * dest1, float * dest2)
    {  
        uint8_t data[12]; // data array to hold accelerometer and gyro x, y, z, data
        uint16_t ii, packet_count, fifo_count;
        int32_t gyro_bias[3]  = {0, 0, 0}, accel_bias[3] = {0, 0, 0};

        // reset device
        writeRegister(PWR_MGMT_1, 0x80); // Write a one to bit 7 reset bit; toggle reset device
        delay(100);

        // get stable time source; Auto select clock source to be PLL gyroscope reference if ready 
        // else use the internal oscillator, bits 2:0 = 001
        writeRegister(PWR_MGMT_1, 0x01);  
        writeRegister(PWR_MGMT_2, 0x00);
        delay(200);                                    

        // Configure device for bias calculation
        writeRegister(INT_ENABLE, 0x00);   // Disable all interrupts
        writeRegister(FIFO_EN, 0x00);      // Disable FIFO
        writeRegister(PWR_MGMT_1, 0x00);   // Turn on internal clock source
        writeRegister(I2C_MST_CTRL, 0x00); // Disable I2C master
        writeRegister(USER_CTRL, 0x00);    // Disable FIFO and I2C master modes
        writeRegister(USER_CTRL, 0x0C);    // Reset FIFO and DMP
        delay(15);

        // Configure MPU6050 gyro and accelerometer for bias calculation
        writeRegister(CONFIG, 0x01);      // Set low-pass filter to 188 Hz
        writeRegister(SMPLRT_DIV, 0x00);  // Set sample rate to 1 kHz
        writeRegister(GYRO_CONFIG, 0x00);  // Set gyro full-scale to 250 degrees per second, maximum sensitivity
        writeRegister(ACCEL_CONFIG, 0x00); // Set accelerometer full-scale to 2 g, maximum sensitivity

        uint16_t  gyrosensitivity  = 131;   // = 131 LSB/degrees/sec
        uint16_t  accelsensitivity = 16384;  // = 16384 LSB/g

        // Configure FIFO to capture accelerometer and gyro data for bias calculation
        writeRegister(USER_CTRL, 0x40);   // Enable FIFO  
        writeRegister(FIFO_EN, 0x78);     // Enable gyro and accelerometer sensors for FIFO  (max size 512 bytes in MPU-9150)
        delay(40); // accumulate 40 samples in 40 milliseconds = 480 bytes

        // At end of sample accumulation, turn off FIFO sensor read
        writeRegister(FIFO_EN, 0x00);        // Disable gyro and accelerometer sensors for FIFO
        readRegisters(FIFO_COUNTH, 2, &data[0]); // read FIFO sample count
        fifo_count = ((uint16_t)data[0] << 8) | data[1];
        packet_count = fifo_count/12;// How many sets of full gyro and accelerometer data for averaging

        for (ii = 0; ii < packet_count; ii++) {
            int16_t accel_temp[3] = {0, 0, 0}, gyro_temp[3] = {0, 0, 0};
            readRegisters(FIFO_R_W, 12, &data[0]); // read data for averaging
            accel_temp[0] = (int16_t) (((int16_t)data[0] << 8) | data[1]  ) ;  // Form signed 16-bit integer for each sample in FIFO
            accel_temp[1] = (int16_t) (((int16_t)data[2] << 8) | data[3]  ) ;
            accel_temp[2] = (int16_t) (((int16_t)data[4] << 8) | data[5]  ) ;    
            gyro_temp[0]  = (int16_t) (((int16_t)data[6] << 8) | data[7]  ) ;
            gyro_temp[1]  = (int16_t) (((int16_t)data[8] << 8) | data[9]  ) ;
            gyro_temp[2]  = (int16_t) (((int16_t)data[10] << 8) | data[11]) ;

            accel_bias[0] += (int32_t) accel_temp[0]; // Sum individual signed 16-bit biases to get accumulated signed 32-bit biases
            accel_bias[1] += (int32_t) accel_temp[1];
            accel_bias[2] += (int32_t) accel_temp[2];
            gyro_bias[0]  += (int32_t) gyro_temp[0];
            gyro_bias[1]  += (int32_t) gyro_temp[1];
            gyro_bias[2]  += (int32_t) gyro_temp[2];
                
        }
        accel_bias[0] /= (int32_t) packet_count; // Normalize sums to get average count biases
        accel_bias[1] /= (int32_t) packet_count;
        accel_bias[2] /= (int32_t) packet_count;
        gyro_bias[0]  /= (int32_t) packet_count;
        gyro_bias[1]  /= (int32_t) packet_count;
        gyro_bias[2]  /= (int32_t) packet_count;
            
        if(accel_bias[2] > 0L) {accel_bias[2] -= (int32_t) accelsensitivity;}  // Remove gravity from the z-axis accelerometer bias calculation
        else {accel_bias[2] += (int32_t) accelsensitivity;}
           
        // Construct the gyro biases for push to the hardware gyro bias registers, which are reset to zero upon device startup
        data[0] = (-gyro_bias[0]/4  >> 8) & 0xFF; // Divide by 4 to get 32.9 LSB per deg/s to conform to expected bias input format
        data[1] = (-gyro_bias[0]/4)       & 0xFF; // Biases are additive, so change sign on calculated average gyro biases
        data[2] = (-gyro_bias[1]/4  >> 8) & 0xFF;
        data[3] = (-gyro_bias[1]/4)       & 0xFF;
        data[4] = (-gyro_bias[2]/4  >> 8) & 0xFF;
        data[5] = (-gyro_bias[2]/4)       & 0xFF;
          
        // Push gyro biases to hardware registers
        writeRegister(XG_OFFSET_H, data[0]);
        writeRegister(XG_OFFSET_L, data[1]);
        writeRegister(YG_OFFSET_H, data[2]);
        writeRegister(YG_OFFSET_L, data[3]);
        writeRegister(ZG_OFFSET_H, data[4]);
        writeRegister(ZG_OFFSET_L, data[5]);
          
        // Output scaled gyro biases for display in the main program
        dest1[0] = (float) gyro_bias[0]/(float) gyrosensitivity;  
        dest1[1] = (float) gyro_bias[1]/(float) gyrosensitivity;
        dest1[2] = (float) gyro_bias[2]/(float) gyrosensitivity;

        // Construct the accelerometer biases for push to the hardware accelerometer bias registers. These registers contain
        // factory trim values which must be added to the calculated accelerometer biases; on boot up these registers will hold
        // non-zero values. In addition, bit 0 of the lower byte must be preserved since it is used for temperature
        // compensation calculations. Accelerometer bias registers expect bias input as 2048 LSB per g, so that
        // the accelerometer biases calculated above must be divided by 8.

        int32_t accel_bias_reg[3] = {0, 0, 0}; // A place to hold the factory accelerometer trim biases
        readRegisters(XA_OFFSET_H, 2, &data[0]); // Read factory accelerometer trim values
        accel_bias_reg[0] = (int32_t) (((int16_t)data[0] << 8) | data[1]);
        readRegisters(YA_OFFSET_H, 2, &data[0]);
        accel_bias_reg[1] = (int32_t) (((int16_t)data[0] << 8) | data[1]);
        readRegisters(ZA_OFFSET_H, 2, &data[0]);
        accel_bias_reg[2] = (int32_t) (((int16_t)data[0] << 8) | data[1]);

        uint32_t reserved_bits[3]; // Define array to hold mask bit for each accelerometer bias axis

//        Serial.print("accel_bias_reg BEFORE: [");
//        Serial.print(accel_bias_reg[0]);
//        Serial.print(",\t");
//        Serial.print(accel_bias_reg[1]);
//        Serial.print(",\t");
//        Serial.print(accel_bias_reg[2]);
//        Serial.println("]");


        for(ii = 0; ii < 3; ii++) {
            reserved_bits[ii] = accel_bias_reg[ii] & 1; // If temperature compensation bit is set, record that fact in mask_bit
            accel_bias_reg[ii] >>= 1;
        }
          
          // Construct total accelerometer bias, including calculated average accelerometer bias from above
        accel_bias_reg[0] -= (accel_bias[0]/8); // Subtract calculated averaged accelerometer bias scaled to 2048 LSB/g (16 g full scale)
        accel_bias_reg[1] -= (accel_bias[1]/8);
        accel_bias_reg[2] -= (accel_bias[2]/8);

        for(ii = 0; ii < 3; ii++) {
            accel_bias_reg[ii] = (accel_bias_reg[ii] << 1) | reserved_bits[ii];
        }

        data[0] = (accel_bias_reg[0] >> 8) & 0xFF;
        data[1] = (accel_bias_reg[0])      & 0xFF;
        data[2] = (accel_bias_reg[1] >> 8) & 0xFF;
        data[3] = (accel_bias_reg[1])      & 0xFF;
        data[4] = (accel_bias_reg[2] >> 8) & 0xFF;
        data[5] = (accel_bias_reg[2])      & 0xFF;
         
        // Apparently this is not working for the acceleration biases in the MPU-9250
        // Are we handling the temperature correction bit properly?
        // Push accelerometer biases to hardware registers
//        writeRegister(XA_OFFSET_H, data[0]);
//        writeRegister(XA_OFFSET_L, data[1]);
//        writeRegister(YA_OFFSET_H, data[2]);
//        writeRegister(YA_OFFSET_L, data[3]);
//        writeRegister(ZA_OFFSET_H, data[4]);
//        writeRegister(ZA_OFFSET_L, data[5]);
//
//        readRegisters(XA_OFFSET_H, 2, &data[0]); // Read factory accelerometer trim values
//        accel_bias_reg[0] = (int32_t) (((int16_t)data[0] << 8) | data[1]);
//        readRegisters(YA_OFFSET_H, 2, &data[0]);
//        accel_bias_reg[1] = (int32_t) (((int16_t)data[0] << 8) | data[1]);
//        readRegisters(ZA_OFFSET_H, 2, &data[0]);
//        accel_bias_reg[2] = (int32_t) (((int16_t)data[0] << 8) | data[1]);
//
//        Serial.print("accel_bias_reg AFTER: [");
//        Serial.print(accel_bias_reg[0]);
//        Serial.print(",\t");
//        Serial.print(accel_bias_reg[1]);
//        Serial.print(",\t");
//        Serial.print(accel_bias_reg[2]);
//        Serial.println("]");

        // Output scaled accelerometer biases for display in the main program
        dest2[0] = (float)accel_bias[0]/(float)accelsensitivity; 
        dest2[1] = (float)accel_bias[1]/(float)accelsensitivity;
        dest2[2] = (float)accel_bias[2]/(float)accelsensitivity;
    }


    void readData(float* sensor_data){
        uint8_t buff[21];
        // grab the data from the MPU9250
        readRegisters(ACCEL_OUT, sizeof(buff), &buff[0], true); 
        int16_t accel[3];
        // combine into 16 bit values
        to16bit(&buff[0], &accel[0], 3);

        int16_t gyro[3];
        to16bit(&buff[8], &gyro[0], 3);

        int16_t mag[3];
        if( !(buff[20] & 0x08) ) { // check for overflow
            to16bit(&buff[14], &mag[0], 3, true);
        }
        else{
            Serial.println(F("WARNING: Magnetometer overflow."));
            mag[0] = 0;  
            mag[2] = 0;
            mag[1] = 0;
        }

        int16_t temperature;
        to16bit(&buff[6], &temperature);

        // accel
        sensor_data[0] = ((float) accel[0]) * accelScale; 
        sensor_data[1] = ((float) accel[1]) * accelScale;
        sensor_data[2] = ((float) accel[2]) * accelScale;

        // gyro
        sensor_data[3] = ((float) gyro[0]) * gyroScale; 
        sensor_data[4] = ((float) gyro[1]) * gyroScale;
        sensor_data[5] = ((float) gyro[2]) * gyroScale;

        // magnet
        sensor_data[6] = (((float) mag[1])  * _mag._magCalibration[1] - _mag._magBias[1]) * _mag._magScale[1]; 
        sensor_data[7] = (((float) mag[0])  * _mag._magCalibration[0] - _mag._magBias[0]) * _mag._magScale[0];
        sensor_data[8] = -(((float) mag[2]) * _mag._magCalibration[2] - _mag._magBias[2]) * _mag._magScale[2];

        // temperature
        sensor_data[9] = (( ((float) temperature) - tempOffset )/tempScale) + tempOffset; 
    }


    /********************************************************************
    UTILS
    *********************************************************************/
    void writeRegisterBit(uint8_t address, byte n, bool v = true, uint8_t delay_ms = 10){
        byte settings = readRegister(address);
        setNthBit(&settings, n, v);
        writeRegister(address, settings, delay_ms);
    }

    bool writeRegister(uint8_t address, uint8_t data, uint8_t delay_ms = 1){
        bool res = _bus->writeByte(MPU9250_I2C_ADDRESS, address, data);
        delay(delay_ms);
        return res;
    }

    void readRegisters(uint8_t address, uint8_t count, uint8_t* dest, bool fast = false){
        _bus->readBytes(MPU9250_I2C_ADDRESS, address, count, dest, fast);
    }

    byte readRegister(uint8_t address){
        return _bus->readByte(MPU9250_I2C_ADDRESS, address);
    }
};

#endif