#include "Raspi_Servo.h"

Raspi_Servo::Raspi_Servo(Raspi_MotorHAT* controller, uint8_t num){
#ifdef RaspiMotorHAT_DEBUG
    Log.trace("START Raspi_Servo::Raspi_Servo(num=%d)\n",num );
#endif
    _MC = controller;
    _servonum = num;
    _minPulse=0;
    _maxPulse=0;
    _minDegree=0;
    _maxDegree=0;

#ifdef RaspiMotorHAT_DEBUG
    Log.trace("END Raspi_Servo::Raspi_Servo(num=%d)\n",num);
#endif
}

bool Raspi_Servo::setPulseRange(uint16_t minPulse,uint16_t maxPulse){
#ifdef RaspiMotorHAT_DEBUG
    Log.trace("START  Raspi_Servo::setPulseRange(minPulse=%d,maxPulse=%d), servo=%d\n",minPulse,maxPulse,_servonum);
#endif
    //check range is OK
    if (minPulse>=maxPulse){
#ifdef RaspiMotorHAT_DEBUG
    Log.error("       Range ERROR (%d >= %d), isOK=0\n",minPulse,maxPulse);
#endif
         return false;
    }

    _minPulse=minPulse;
    _maxPulse=maxPulse;
#ifdef RaspiMotorHAT_DEBUG
    Log.trace("END  Raspi_Servo::setPulseRange(minPulse=%d,maxPulse=%d), servo=%d, isOK=1\n",_minPulse,_maxPulse,_servonum);
#endif
    return true;
}

bool Raspi_Servo::setDegreeRange(uint16_t minDegree,uint16_t maxDegree){
#ifdef RaspiMotorHAT_DEBUG
    Log.trace("START  Raspi_Servo::setDegreeRange(minDegree=%d,maxDegree=%d), servo=%d\n",minDegree,maxDegree,_servonum);
#endif
    //check range is OK
    if (minDegree>=maxDegree){
#ifdef RaspiMotorHAT_DEBUG
    Log.error("       Range ERROR (%d >= %d), isOK=0\n",minDegree,maxDegree);
#endif
         return false;
    }

    _minDegree=minDegree;
    _maxDegree=maxDegree;
#ifdef RaspiMotorHAT_DEBUG
    Log.trace("END  Raspi_Servo::setDegreeRange(minDegree=%d,maxDegree=%d), servo=%d, isOK=1\n",_minDegree,_maxDegree,_servonum);
#endif
    return true;
}

bool Raspi_Servo::setDegrees(uint16_t degrees){
#ifdef RaspiMotorHAT_DEBUG
    Log.trace("START  Raspi_Servo::setDegrees(degrees=%d)\n",degrees);
#endif
    //check range is OK
    if (_minDegree>degrees || _maxDegree<degrees){
#ifdef RaspiMotorHAT_DEBUG
        Log.trace("   Raspi_Servo::setDegrees: WARN Rang ERROR");
#endif
        if (_minDegree>degrees) degrees=_minDegree;
        if (_maxDegree<degrees) degrees=_maxDegree;
    }

    bool isOK;
    uint16_t pulse = map(degrees, 0, 180, _minPulse, _maxPulse);
#ifdef RaspiMotorHAT_DEBUG
    Log.trace("      Raspi_Servo::setDegrees: Calculated %d° => %d Pulses\n",degrees,pulse);
#endif
    isOK=setPulse(pulse);

#ifdef RaspiMotorHAT_DEBUG
    Log.trace("END   Raspi_Servo::setDegrees(degrees=%d), servo=%d isOK=%d\n",degrees,_servonum,isOK);
#endif
    return isOK;
}

//setServoPulse(0, 0.001); should position the servo at about 0 degrees
//setServoPulse(0, 0.0015); should position the servo at about 90 degrees
//setServoPulse(0, 0.002); should position the servo at about 180 degrees
bool Raspi_Servo::setPulse(uint16_t pulse){
#ifdef RaspiMotorHAT_DEBUG
    Log.trace("START Raspi_Servo::setServoPulse(pulse=%d), servo=%d\n",pulse,_servonum );
#endif

    bool isOK=true;
    if (!_MC->_pwm->setPWMFreq(60)) isOK=false; // Analog servos run at ~60 Hz updates
	if (isOK && !_MC->_pwm->setPWM(_servonum, 0,pulse)) isOK=false;

#ifdef RaspiMotorHAT_DEBUG    
    Log.trace("END Raspi_Servo::setServoPulse(pulse=%d), servo==%d, isOK=%d\n",pulse,_servonum,isOK);
#endif
    return isOK;
}