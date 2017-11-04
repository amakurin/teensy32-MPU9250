#ifndef MPU9250_h
#define MPU9250_h

#include "Arduino.h"
#include "bus.h"
#include "ak8963.h"
#include "utils.h"

class MPU9250 {
public:

    enum Algorythm
    {
        MADGWICK,
        MAHONY,
        DMP,
        EKF,
        NONE
    };

    enum GyroRes
    {
        DPS250, 
        DPS500, 
        DPS1000, 
        DPS2000
    };

    enum AccelRes
    {
        G2, 
        G4, 
        G8, 
        G16
    };

    enum GyroDLPF
    {
        BW_8800Hz, 
        BW_3600Hz, 
        BW_250Hz, 
        BW_184Hz, 
        BW_92Hz, 
        BW_41Hz, 
        BW_20Hz, 
        BW_10Hz, 
        BW_5Hz
    };

    enum AccelDLPF
    {
        BW_1046Hz, 
        BW_218Hz, 
        BW_99Hz, 
        BW_44Hz, 
        BW_21Hz, 
        BW_10_2Hz, 
        BW_5_05Hz
    };

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
    static const uint8_t I2C_SLV0_DO    = 0x63;
    static const uint8_t EXT_SENS_DATA_00 = 0x49;

    static const uint8_t I2C_READ_FLAG  = 0x80;
   
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
    static constexpr float d2r = 3.14159265359f/180.0f;
    static constexpr float tempScale = 333.87f;
    static constexpr float tempOffset = 21.0f;

    bool _interrupt = false;
    bool _interrupts_enabled = false;
    Algorythm  _algorythm ;
    GyroRes  _gyroRes     ;
    AccelRes _accelRes    ;
    GyroDLPF   _gyroDLPF  ;
    AccelDLPF  _accelDLPF ;

    float _gyroScale;
    uint8_t _gyroRegConfig;
    uint8_t _gyroDLPFRegConfig;
    uint8_t _gyroDLPFFCHOISEConfig;
    
    float _accelScale;
    uint8_t _accelRegConfig;
    uint8_t _accelDLPFRegConfig;
    uint8_t _accelDLPFFCHOISEConfig;

    Bus* _bus;
    AK8963 _mag;
    MPU9250 (Bus* bus) : _bus(bus), _mag(_bus) {
        _bus->begin();

        setAlgorythm(MADGWICK   );
        setGyroRes  (DPS250     );
        setAccelRes (G2         );
        setGyroDLPF (BW_3600Hz  );
        setAccelDLPF(BW_1046Hz  );
    }

    void setAlgorythm(Algorythm v){
        _algorythm = v;
    }

    void setGyroRes(GyroRes v){
        _gyroRes = v;
        switch(_gyroRes) {
            case DPS250 : _gyroScale = 250.0f ; _gyroRegConfig = 0x00; break;
            case DPS500 : _gyroScale = 500.0f ; _gyroRegConfig = 0x08; break;
            case DPS1000: _gyroScale = 1000.0f; _gyroRegConfig = 0x10; break;
            case DPS2000: _gyroScale = 2000.0f; _gyroRegConfig = 0x18; break;
        }
        _gyroScale = _gyroScale /32767.5f * d2r;

    }

    void setAccelRes(AccelRes v){
        _accelRes = v;
        switch(_accelRes) {
            case G2 : _accelScale = 2.0f ; _accelRegConfig = 0x00; break;
            case G4 : _accelScale = 4.0f ; _accelRegConfig = 0x08; break;
            case G8 : _accelScale = 8.0f ; _accelRegConfig = 0x10; break;
            case G16: _accelScale = 16.0f; _accelRegConfig = 0x18; break;
        }
        _accelScale = G * _accelScale /32767.5f;
    }

    void setGyroDLPF(GyroDLPF v){
        _gyroDLPF = v;
        _gyroDLPFFCHOISEConfig = 0x00;
        switch (_gyroDLPF){
            case BW_8800Hz : _gyroDLPFRegConfig = 0x00; _gyroDLPFFCHOISEConfig = 0x03; break;
            case BW_3600Hz : _gyroDLPFRegConfig = 0x00; _gyroDLPFFCHOISEConfig = 0x02; break;
            case BW_250Hz  : _gyroDLPFRegConfig = 0x00; break;
            case BW_184Hz  : _gyroDLPFRegConfig = 0x01; break;
            case BW_92Hz   : _gyroDLPFRegConfig = 0x02; break;
            case BW_41Hz   : _gyroDLPFRegConfig = 0x03; break;
            case BW_20Hz   : _gyroDLPFRegConfig = 0x04; break;
            case BW_10Hz   : _gyroDLPFRegConfig = 0x05; break;
            case BW_5Hz    : _gyroDLPFRegConfig = 0x06; break;
        } 
    }

    void setAccelDLPF(AccelDLPF v){
        _accelDLPF = v;
        _accelDLPFFCHOISEConfig = 0x00;
        switch (_accelDLPF){
            case BW_1046Hz    : _accelDLPFRegConfig = 0x00; _accelDLPFFCHOISEConfig = 0x08; break;
            case BW_218Hz     : _accelDLPFRegConfig = 0x01; break;
            case BW_99Hz      : _accelDLPFRegConfig = 0x02; break;
            case BW_44Hz      : _accelDLPFRegConfig = 0x03; break;
            case BW_21Hz      : _accelDLPFRegConfig = 0x04; break;
            case BW_10_2Hz    : _accelDLPFRegConfig = 0x05; break;
            case BW_5_05Hz    : _accelDLPFRegConfig = 0x06; break;
        } 
    }

    void setInterrupt(){
        _interrupt = true;
    }

    bool readInterrupt(){
        if (!_interrupts_enabled) return true;
        bool result = false;
        cli();
        result = _interrupt;
        _interrupt = false;
        sei();
        return result;
    }

    void toBypassMode() {
        writeRegisterBit(PWR_MGMT_1, H_RESET);      // Power reset
        writeRegister(INT_PIN_CFG, 0x02);           // Enable I2C bypass, disable FSYNC
        writeRegisterBit(USER_CTRL, I2C_MST_EN, 0); // Disable I2C Master Mode
    }

    void switchInterrupts(bool enable){
        _interrupts_enabled = enable;
    }

    void setup() {
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

        writeRegister(PWR_MGMT_1, 0x01); // Auto selects the best available clock source – PLL if ready, else use the Internal oscillator

        // ORDER MATTERS: First - enable master, then - setup mag  
        writeRegister(I2C_MST_CTRL, I2C_MST_CLK);    // set i2c to 400Hz
        writeRegisterBit(USER_CTRL, I2C_IF_DIS );    // Disable I2C Slave
        writeRegisterBit(USER_CTRL, I2C_MST_EN );    // Enable I2C Master Mode

        AK8963Setup();
        delay(20);

        writeRegister(CONFIG, _gyroDLPFRegConfig);
        writeRegister(GYRO_CONFIG, _gyroRegConfig|_gyroDLPFFCHOISEConfig);
        writeRegister(ACCEL_CONFIG, _accelRegConfig);
        writeRegister(ACCEL_CONFIG2, _accelDLPFRegConfig|_accelDLPFFCHOISEConfig);
        delay(20);

        if (_interrupts_enabled) {
            writeRegisterBit(INT_PIN_CFG, 4);   // clear interrupt on any read
            writeRegisterBit(INT_PIN_CFG, 5);   // INT pin level held until interrupt status is cleared        
            writeRegisterBit(INT_ENABLE, 0);    // RAW_RDY_EN
        }
    }

    // Function which accumulates gyro and accelerometer data after device initialization. It calculates the average
    // of the at-rest readings and then loads the resulting offsets into accelerometer and gyro bias registers.
    void calibrate(float * dest1, float * dest2)
    {  
        uint8_t data[12]; // data array to hold accelerometer and gyro x, y, z, data
        uint16_t ii, packet_count, fifo_count;
        int32_t gyro_bias[3]  = {0, 0, 0}, accel_bias[3] = {0, 0, 0};

        // reset device
        writeRegisterBit(PWR_MGMT_1, H_RESET); // Write a one to bit 7 reset bit; toggle reset device
        delay(20);

        // get stable time source; Auto select clock source to be PLL gyroscope reference if ready 
        // else use the internal oscillator, bits 2:0 = 001
        writeRegister(PWR_MGMT_1, 0x01);  
        writeRegister(PWR_MGMT_2, 0x00);
        delay(20);                                    

        // Configure device for bias calculation
        writeRegister(INT_ENABLE, 0x00);   // Disable all interrupts
        writeRegister(FIFO_EN, 0x00);      // Disable FIFO
        writeRegister(PWR_MGMT_1, 0x00);   // Turn on internal clock source
        writeRegister(I2C_MST_CTRL, 0x00); // Disable I2C master
        writeRegister(USER_CTRL, 0x00);    // Disable FIFO and I2C master modes
        writeRegister(USER_CTRL, 0x04);    // Reset FIFO 
        delay(20);

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
        
        writeRegister(INT_ENABLE, 0x00);   // Disable all interrupts
        writeRegister(FIFO_EN, 0x00);      // Disable FIFO
        writeRegister(USER_CTRL, 0x00);    // Disable FIFO and I2C master modes
        writeRegister(USER_CTRL, 0x04);    // Reset FIFO 
        delay(20);
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
        sensor_data[0] = ((float) accel[0]) * _accelScale; 
        sensor_data[1] = ((float) accel[1]) * _accelScale;
        sensor_data[2] = ((float) accel[2]) * _accelScale;

        // gyro
        sensor_data[3] = ((float) gyro[0]) * _gyroScale; 
        sensor_data[4] = ((float) gyro[1]) * _gyroScale;
        sensor_data[5] = ((float) gyro[2]) * _gyroScale;

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

    //************************************************************************ 
    // Magnetometer AK8963 registers Read\Write using mpu I2C Master features
    // We'll use these when there is no ability to communicate directly by i2c
    //************************************************************************ 
    bool writeAK8963Register(uint8_t address, uint8_t data){
        uint8_t count = 1;
        uint8_t buff[1];

        writeRegister(I2C_SLV0_ADDR, AK8963::I2C_ADDRESS); // set slave 0 to the AK8963 and set for write
        writeRegister(I2C_SLV0_REG, address);              // set the register to the desired AK8963 sub address
        writeRegister(I2C_SLV0_DO,data);                   // store the data for write
        writeRegister(I2C_SLV0_CTRL, I2C_SLV0_EN | count);  // enable I2C and send 1 byte

        // read the register and confirm
        readAK8963Registers(address, sizeof(buff), &buff[0]);

        if(buff[0] == data) {
            return true;
        }
        else{
            return false;
        }
    }

    void readAK8963Registers(uint8_t address, uint8_t count, uint8_t* dest){
        writeRegister(I2C_SLV0_ADDR, AK8963::I2C_ADDRESS | I2C_READ_FLAG); // set slave 0 to the AK8963 and set for read
        writeRegister(I2C_SLV0_REG, address);               // set the register to the desired AK8963 sub address
        writeRegister(I2C_SLV0_CTRL, I2C_SLV0_EN | count);  // enable I2C and request the bytes
        delayMicroseconds(100);                             // takes some time for these registers to fill
        readRegisters(EXT_SENS_DATA_00,count,dest);         // read the bytes off the MPU9250 EXT_SENS_DATA registers
    }

    void AK8963Setup(){
        writeAK8963Register(AK8963::CNTL1, AK8963::CNTL1_PWR_DOWN); // Power down magnetometer  
        writeAK8963Register(AK8963::CNTL1, AK8963::CNTL1_FUSE); // Enter Fuse ROM access mode

        // Extract the factory calibration for each magnetometer axis
        uint8_t rawData[3];  
        readAK8963Registers(AK8963::ASAX, 3, rawData);  // Read the x-, y-, and z-axis calibration values
        float magnetometerResolution = 4912.0f / 32760.0f; // micro Tesla
        _mag._magCalibration[0] = ((float)(rawData[0] - 128)/256. + 1.) * magnetometerResolution; 
        _mag._magCalibration[1] = ((float)(rawData[1] - 128)/256. + 1.) * magnetometerResolution;  
        _mag._magCalibration[2] = ((float)(rawData[2] - 128)/256. + 1.) * magnetometerResolution; 

        writeAK8963Register(AK8963::CNTL1, AK8963::CNTL1_PWR_DOWN); // Power down magnetometer  
        delay(100);
        writeAK8963Register(AK8963::CNTL1, AK8963::CNTL1_CONT_MEAS_2); // Set mode to 16 bit resolution at 100Hz continuous reading
        
        // By this read we setup auto slave reading and actually read for the first time.
        uint8_t buff[7];
        readAK8963Registers(AK8963::HXL, 7, buff);
    }

    void AK8963calibrate(float* bias, float* scale){
        uint16_t ii = 0, sample_count = 1500; // at 100 Hz ODR, new mag data is available every 10 ms
        int32_t int32_bias[3] = {0, 0, 0}, int32_scale[3] = {0, 0, 0};
        int16_t mag_max[3] = {-32767, -32767, -32767}, mag_min[3] = {32767, 32767, 32767}, mag_temp[3] = {0, 0, 0};
        delay(4000);

        // shoot for ~fifteen seconds of mag data
        for(ii = 0; ii < sample_count; ii++) {
            uint8_t buffer[7];
            readAK8963Registers(AK8963::HXL, 7, buffer);  
            to16bit(&buffer[0], mag_temp, 3, true);
            if (!_mag.isOverflow(buffer[6])){  // Read the mag data   
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

        bias[0] = (float) int32_bias[0] * _mag._magCalibration[0];  // save mag biases in uT for main program
        bias[1] = (float) int32_bias[1] * _mag._magCalibration[1];   
        bias[2] = (float) int32_bias[2] * _mag._magCalibration[2];  

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

};

#endif