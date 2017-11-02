#include "MPU9250.h"
#include "i2cbus.h"
#include "spibus.h"
#include "commands.h"
#include <memory>

#define PIN_INTERRUPT 22
#define ENABLE_INTERRUPTS 0


SPIBus spibus;
//I2CBus i2cbus;
MPU9250 mpu9250(&spibus);
byte buffer[64];
std::shared_ptr<BaseCommand> pCommand;

void setup() {
    Serial.begin(115200);
    mpu9250.switchInterrupts(ENABLE_INTERRUPTS);
    if (ENABLE_INTERRUPTS) {
        pinMode(PIN_INTERRUPT, INPUT);
        attachInterrupt(PIN_INTERRUPT, isrService, RISING);
    }
}

void isrService()
{
    mpu9250.setInterrupt();
}

void loop() {
    int n = RawHID.recv(buffer, 0); // 0 timeout = do not wait
    if (n > 0) {
        USBCommand cmd_code = BaseCommand::getCommandCode(buffer);
        switch (cmd_code){
            case CMD_START_SENSORS  : pCommand.reset(new StartSensorsCommand    (&mpu9250, buffer)); break;
            case CMD_STOP           : pCommand.reset(new GenericStopCommand     (&mpu9250, buffer)); break;
            case CMD_MAG_CALIB      : pCommand.reset(new CalibrateMagnetometerCommand(&mpu9250, buffer)); break;
            case CMD_READ_REGS      : pCommand.reset(new ReadRegistersCommand   (&mpu9250, buffer)); break;
            default: 
                Serial.print(F("Unknown command received: "));
                Serial.println(cmd_code);
        }
        pCommand.get()->setup();
    }

    if (pCommand){
        bool continue_exec = pCommand.get()->exec();
        if (!continue_exec) pCommand.reset();
    }
}

