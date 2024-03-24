#ifndef ServoMotor_H
#define ServoMotor_H

#include <Servo.h> 
#include <Arduino.h>
#include "Raspi_MotorHAT.h"
#include "ArduinoLog.h"

class ServoMotor
{
    public:
        enum driver {L298, BTS7960, ESC,RASPIMOTORHAT};
        ServoMotor(driver ServoMotor_driver, int pwm_channel, int minPulse, int maxPulse, int minDegree, int maxDegree, int initDegree);
        bool initialize();
        bool setPulse(int pwm);
        bool setDegrees(int degrees);
        bool setPulseRange(int minPulse, int maxPulse);
        bool isConnected();
        bool isConnected(uint8_t address);

    private:
        Servo ServoMotor_;
        driver ServoMotor_driver_;
        int pwm_channel_;
        int ServoMotor_minPulse_;
        int ServoMotor_maxPulse_;
        int ServoMotor_minDegree_;
        int ServoMotor_maxDegree_;
        int ServoMotor_initDegree_;
};

#endif
