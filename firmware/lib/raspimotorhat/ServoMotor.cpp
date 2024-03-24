#include "ServoMotor.h"
            
#ifndef ServoMotorHAT_AVAILABLE
Raspi_MotorHAT ServoMotorHAT(0x6F, 160);
#define ServoMotorHAT_AVAILABLE
#endif


ServoMotor::ServoMotor(driver ServoMotor_driver, int pwm_channel, int minPulse, int maxPulse, int minDegree, int maxDegree, int initDegree):
    ServoMotor_driver_(ServoMotor_driver),
    pwm_channel_(pwm_channel),
    ServoMotor_minPulse_(minPulse),
    ServoMotor_maxPulse_(maxPulse),
    ServoMotor_minDegree_(minDegree),
    ServoMotor_maxDegree_(maxDegree),
    ServoMotor_initDegree_(initDegree)
{      
    //Log.setLevel(LOG_LEVEL_VERBOSE);
#ifdef RASPIMotorHAT_DEBUG    
    Log.trace("START ServoMotor::ServoMotor(pwm_channel=%d, minPulse=%d, maxPulse=%d, initPulse=%d)\n", pwm_channel, minPulse, maxPulse, initPulse);
#endif 
    initialize();
#ifdef RASPIMotorHAT_DEBUG
    Log.trace("END ServoMotor::ServoMotor()\n");
#endif
}

bool ServoMotor::initialize() {
    //Log.setLevel(LOG_LEVEL_VERBOSE);
    bool isOK=true;
    
#ifdef RASPIMotorHAT_DEBUG
    Log.trace("START ServoMotor::initialize()\n");
#endif
    switch (ServoMotor_driver_)
    {
        case RASPIMOTORHAT:
            
            if (!ServoMotorHAT.initialize()) isOK=false;
            if (isOK && !ServoMotorHAT.getServo(pwm_channel_)->setPulseRange(ServoMotor_minPulse_,ServoMotor_maxPulse_)) isOK=false;
            if (isOK && !ServoMotorHAT.getServo(pwm_channel_)->setDegreeRange(ServoMotor_minDegree_,ServoMotor_maxDegree_)) isOK=false;
            //ensure that the Servo "pwm_channel" is in neutral state during bootup
            if (isOK && !ServoMotorHAT.getServo(pwm_channel_)->setDegrees(ServoMotor_initDegree_)) isOK=false;
            break;
    }
#ifdef RASPIMotorHAT_DEBUG
    Log.trace("END ServoMotor::initialize(), isOK=%d\n",isOK);
#endif
    return isOK;
}

bool ServoMotor::isConnected() {
    if (ServoMotor_driver_==RASPIMOTORHAT)
      return ServoMotorHAT.isConnected();
    else
      return true;
}

bool ServoMotor::isConnected(uint8_t address) {
    if (ServoMotor_driver_==RASPIMOTORHAT)
      return ServoMotorHAT.isConnected(address);
    else
      return true;
}

bool ServoMotor::setPulse(int pulse)
{
    switch (ServoMotor_driver_)
    {           
        case RASPIMOTORHAT:
            if (!ServoMotorHAT.isConnected() && !ServoMotorHAT.initialize()) {
#ifdef RASPIMotorHAT_DEBUG
               Log.error("The Servo %d is not initialized\n",pwm_channel_);
#endif
               return false;
            }

//#ifdef RASPIMotorHAT_DEBUG
    Log.trace("    ServoMotor::setPulse: Checking ServoMotor_minPulse_(%d) <= pulse(%d) <= ServoMotor_maxPulse_(%d)\n",ServoMotor_minPulse_,pulse,ServoMotor_maxPulse_);
//#endif
            if (abs(pulse)>ServoMotor_maxPulse_) {
               if (pulse>0)
                  pulse=ServoMotor_maxPulse_;
               else
                  pulse=-ServoMotor_maxPulse_;
            }else
               if (abs(pulse)<=ServoMotor_minPulse_) pulse=0;       // set to 0 to be sure the Servo will not move
                           
            // Set Servo Speed
            if (!ServoMotorHAT.getServo(pwm_channel_)->setPulse(abs(pulse))) return false;
            break;
    }
    return true;
}

bool ServoMotor::setDegrees(int degrees)
{
    switch (ServoMotor_driver_)
    {           
        case RASPIMOTORHAT:
            if (!ServoMotorHAT.isConnected() && !ServoMotorHAT.initialize()) {
#ifdef RASPIMotorHAT_DEBUG
               Log.error("     ServoMotor::setDegrees: The Servo %d is not initialized\n",pwm_channel_);
#endif
               return false;
            }
                          
            // Set Servo Speed
            if (!ServoMotorHAT.getServo(pwm_channel_)->setDegrees(abs(degrees))) return false;
            break;
    }
    return true;
}

bool ServoMotor::setPulseRange(int minPulse, int maxPulse){
    boolean isOK=true;
    switch (ServoMotor_driver_)
    {           
        case RASPIMOTORHAT:
            if (!ServoMotorHAT.isConnected() && !ServoMotorHAT.initialize()) {
#ifdef RASPIMotorHAT_DEBUG
               Log.error("     ServoMotor::setPulseRange: The Servo %d is not initialized\n",pwm_channel_);
#endif
               return false;
            }
                          
            // Set Servo Speed
            if (!ServoMotorHAT.getServo(pwm_channel_)->setPulseRange(minPulse,maxPulse)) {
               ServoMotor_minPulse_=minPulse;
               ServoMotor_maxPulse_=maxPulse;    
            }
            break;
    }
    return true;
}
