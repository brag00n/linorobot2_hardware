#include <Arduino.h>
#include "config.h"
#include "battery.h"


#ifdef USE_INA219
#include <INA219_WE.h>

#define INA219_ADDRESS 0x42
INA219_WE ina219 = INA219_WE(INA219_ADDRESS);

float shuntVoltage_mV = 0.0;
float loadVoltage_V = 0.0;
float busVoltage_V = 0.0;
float current_mA = 0.0;
float power_mW = 0.0;
bool ina219_overflow = false;

void Battery::initBattery(){
  if(!ina219.init()){
    Serial.println("INA219 not connected!");
  }
  ina219.setADCMode(BIT_MODE_9);
  ina219.setPGain(PG_320);
  ina219.setBusRange(BRNG_16);
  ina219.setShuntSizeInOhms(0.01); // used in INA219.
}

void InaDataUpdate(){
  shuntVoltage_mV = ina219.getShuntVoltage_mV();
  busVoltage_V = ina219.getBusVoltage_V();
  current_mA = ina219.getCurrent_mA();
  power_mW = ina219.getBusPower();
  loadVoltage_V  = busVoltage_V + (shuntVoltage_mV/1000);
  ina219_overflow = ina219.getOverflow();
}
#else
void Battery::initBattery() {
#ifdef BATTERY_PIN
    analogReadResolution(12);
#endif
}
#endif

#ifdef BATTERY_PIN
double readVoltage(int pin) {
  int reading = 0;
  int i;
  for (i = 0; i < 4; i++) // smoothing
    reading += analogRead(pin);
  reading /= i;
  return BATTERY_ADJUST(reading);
}
#endif

Battery& Battery::readBattery()
{
  _battery.voltage=0;
  _battery.current=0;
#ifdef BATTERY_PIN
  _battery.voltage = readVoltage(BATTERY_PIN);
#endif
#ifdef USE_INA219
    // read voltage
    InaDataUpdate();
    _battery.voltage = loadVoltage_V;
    _battery.current = -current_mA / 1000; // Amp, minu when discharge
    getBatteryPercentage();
#endif
  return *this;
}

/** https://github.com/rlogiacco/BatterySense/blob/master/Battery.h
 *
 * Symmetric sigmoidal approximation
 * https://www.desmos.com/calculator/7m9lu26vpy
 *
 * c - c / (1 + k*x/v)^3
 */
static inline float sigmoidal(float voltage, float minVoltage, float maxVoltage) {
    // slow
    // float result = 110 - (110 / (1 + pow(1.468 * (voltage - minVoltage)/(maxVoltage - minVoltage), 6)));

    // steep
    // float result = 102 - (102 / (1 + pow(1.621 * (voltage - minVoltage)/(maxVoltage - minVoltage), 8.1)));

    // normal
    float result = 105 - (105 / (1 + pow(1.724 * (voltage - minVoltage)/(maxVoltage - minVoltage), 5.5)));
    return result >= 100 ? 100 : result;
}

/**
 * Asymmetric sigmoidal approximation
 * https://www.desmos.com/calculator/oyhpsu8jnw
 *
 * c - c / [1 + (k*x/v)^4.5]^3
 */
static inline float asigmoidal(float voltage, float minVoltage, float maxVoltage) {
    float result = 101 - (101 / pow(1 + pow(1.33 * (voltage - minVoltage)/(maxVoltage - minVoltage) ,4.5), 3));
    return result >= 100 ? 100 : result;
}

void Battery::getBatteryPercentage()
{
#if defined(BATTERY_MIN) && defined(BATTERY_MAX)
  _battery.percentage = sigmoidal(_battery.voltage, BATTERY_MIN, BATTERY_MAX) / 100;
#ifdef BATTERY_CAP
  _battery.design_capacity = BATTERY_CAP;
  _battery.capacity = BATTERY_CAP * _percentage;
#endif
#endif
}
