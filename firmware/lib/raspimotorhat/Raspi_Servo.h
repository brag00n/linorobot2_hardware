#ifndef _Raspi_Servo_H_
#define _Raspi_Servo_H_

#include <stdio.h>
#include "Raspi_MotorHAT.h"
#include "ArduinoLog.h"

class Raspi_Servo {
    public:
        Raspi_Servo(Raspi_MotorHAT* controller, uint8_t num);
        bool setPulseRange(uint16_t minPulse,uint16_t maxPulse);
        bool setDegreeRange(uint16_t minPulse,uint16_t maxPulse);
        bool setPulse(uint16_t pulse);
        bool setDegrees(uint16_t degrees);
    public:
      static const int CENTER=  0;
    private:
      Raspi_MotorHAT* _MC;
      uint8_t _servonum;
      uint16_t _minPulse;
      uint16_t _maxPulse;
      uint16_t _minDegree;
      uint16_t _maxDegree;
};
        
#endif /* _Raspi_Servo_H_ */