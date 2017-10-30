#ifndef COMMANDS_h
#define COMMANDS_h
#include "MPU9250.h"
#include "filters.h"
#include "utils.h"

enum USBCommand
{
    CMD_START_SENSORS,
    CMD_STOP,
    CMD_MAG_CALIB,
    CMD_READ_REGS
};


class BaseCommand {
public:
    static const uint USB_PACKET_SIZE = 64;
    MPU9250* _mpu9250;
    byte* _buffer;
    USBCommand _cmd_code;
    uint _write_counter;
    BaseCommand(MPU9250* mpu9250, byte* buffer)
        :_mpu9250(mpu9250), _buffer(buffer), _write_counter(0) {
            _cmd_code = getCommandCode(buffer);
        };

    virtual ~BaseCommand(){};

    virtual void setup(){};
    
    virtual bool exec() = 0;

    static USBCommand getCommandCode(byte* data){
        return (USBCommand) data[0];
    }

    static uint getDataLen(byte* data){
        return (uint) data[1];
    }

    static void bufPrint(byte* data, uint data_len = USB_PACKET_SIZE){
        Serial.print(F("Buffer dump: "));
        for (uint i = 0; i< data_len; i++){
            Serial.print(data[i]);
            Serial.print(", ");
        }
        Serial.println();
    }

    uint getDataLen(){
        return getDataLen(_buffer);
    }

    void bufWrite(byte data){
        _buffer[_write_counter++] = data;
    }

    void bufWrite(void* data, uint data_len){
        for (uint i=0; i < data_len; i++){
            _buffer[i + _write_counter] = *((unsigned char*)(data)+i);
        }
        _write_counter += data_len;
    }

    void bufWriteStart(uint data_len, uint final_packet = 0){
        _write_counter = 0;
        bufWrite((byte) _cmd_code);
        bufWrite((byte) data_len);
        bufWrite((byte) final_packet);
    }

    void bufWriteEnd(){
        for (uint i=_write_counter; i < USB_PACKET_SIZE; i++){
            _buffer[i] = 0;
        }
        _write_counter = USB_PACKET_SIZE;
    }

    bool bufSend(){
        bufWriteEnd();
        int n = RawHID.send(_buffer, 100);
        return n > 0;
    }
    void bufPrint(){
        bufPrint(_buffer);
    }

};

class StartSensorsCommand:public BaseCommand {
public:
    enum Algorithm
    {
        MADGWICK,
        MAHONY
    };
    float _eInt[3];
    float _q[4];
    TimeCounter _timeCounter;
    uint _updateCounter;
    uint _sendThre;
    Algorithm _alg;
    StartSensorsCommand(MPU9250* mpu9250, byte* buffer):BaseCommand(mpu9250, buffer){};
    ~StartSensorsCommand(){}

    void setup(){
        bufPrint();
        _mpu9250->setup();
        _eInt[0] = 0.0;
        _eInt[1] = 0.0;
        _eInt[2] = 0.0;
        _q[0] = 0.39;
        _q[1] = 0.0;
        _q[2] = 0.0;
        _q[3] = -0.92;
        _updateCounter = 0;
        uint data_len = getDataLen();
        if (data_len>0) _sendThre = _buffer[2];
        else _sendThre = 100;
        if (data_len>1) _alg = (Algorithm)_buffer[3];
        else _alg = MADGWICK;
    }

    bool exec(){
//        unsigned long t1 = micros();
        float sensor_data[15]; //ax, ay, az, gx, gy, gz, hx, hy, hz, t, qx, qy, qz, qw;
        _mpu9250->readData(&sensor_data[0]);

        auto dt = _timeCounter.update();
//        unsigned long t2 = micros();
        
        // quaternion
        switch (_alg){
            case MADGWICK :        
                MadgwickQuaternionUpdate(sensor_data, _q, dt); break;
            case MAHONY :
                MahonyQuaternionUpdate(sensor_data, _eInt, _q, dt); break;
        }
//        unsigned long t3 = micros();
//Serial.print(F("Read: "));
//Serial.println(t2-t1);
//Serial.print(F("MADGWICK: "));
//Serial.println(t3-t2);
        _updateCounter = (_updateCounter + 1) % _sendThre;
        if (_updateCounter == 0){
            sensor_data[10] = _q[0]; 
            sensor_data[11] = _q[1];
            sensor_data[12] = _q[2];
            sensor_data[13] = _q[3];
            sensor_data[14] = 1/dt;

            uint data_len = sizeof(sensor_data);
            bufWriteStart(data_len);
            bufWrite(sensor_data, data_len);
            bufSend();
            delay(1);
        }
        return true;
    }
};

class GenericStopCommand:public BaseCommand {
public:
    GenericStopCommand(MPU9250* mpu9250, byte* buffer):BaseCommand(mpu9250, buffer){};
    ~GenericStopCommand(){}

    void setup(){
    }

    bool exec() {
        bufWriteStart(1,1);
        bufWrite(1);
        bufSend();
        return false;
    }
};

class CalibrateMagnetometerCommand:public BaseCommand {
public:
    CalibrateMagnetometerCommand(MPU9250* mpu9250, byte* buffer):BaseCommand(mpu9250, buffer){};
    ~CalibrateMagnetometerCommand(){}

    void setup(){
        _mpu9250->toBypassMode();
        _mpu9250->_mag.setup();
        delay(100);
    }

    bool exec() {
        _mpu9250->_mag.calibrate(_mpu9250->_mag._magBias, _mpu9250->_mag._magScale);
        uint bias_len = sizeof(_mpu9250->_mag._magBias), scale_len = sizeof(_mpu9250->_mag._magScale);
        bufWriteStart(bias_len + scale_len, 1);
        bufWrite(_mpu9250->_mag._magBias, bias_len);
        bufWrite(_mpu9250->_mag._magScale, scale_len);
        bufSend();
        return false;
    }
};

class ReadRegistersCommand:public BaseCommand {
public:
    ReadRegistersCommand(MPU9250* mpu9250, byte* buffer):BaseCommand(mpu9250, buffer){};
    ~ReadRegistersCommand(){}

    void setup(){
        uint data_len = getDataLen();
        if ((data_len>0) && _buffer[2])
            _mpu9250->setup();
    }

    bool exec() {
        byte reg_data[127];
        _mpu9250->readRegisters(0x00, sizeof(reg_data), reg_data);
        delay(10);
        bufPrint(reg_data, sizeof(reg_data));
        uint startAddr = 0;
        bufWriteStart(61);
        bufWrite(startAddr);
        bufWrite(reg_data, 60);
        bufSend();
        delay(10);
        startAddr += 60;
        bufWriteStart(61);
        bufWrite(startAddr);
        bufWrite(&reg_data[startAddr], 60);
        bufSend();
        delay(10);
        startAddr += 60;
        bufWriteStart(sizeof(reg_data)+1-startAddr,1);
        bufWrite(startAddr);
        bufWrite(&reg_data[startAddr], sizeof(reg_data)-startAddr);
        bufSend();
        delay(10);
        return false;
    }
};


#endif