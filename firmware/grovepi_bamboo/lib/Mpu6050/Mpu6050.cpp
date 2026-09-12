#include "Mpu6050.h"
#include <Wire.h>

// Registres et echelles MPU6050 (config par defaut : +-2 g, +-250 deg/s).
static const uint8_t  MPU_PWR_MGMT_1 = 0x6B;
static const uint8_t  MPU_ACCEL_XOUT = 0x3B;
static const float    ACC_LSB_PER_G   = 16384.0f;
static const float    GYR_LSB_PER_DPS = 131.0f;
static const float    COMP_ALPHA      = 0.98f;   // poids gyro du filtre complementaire
static const float    RAD2DEG         = 57.29578f;

bool Mpu6050::begin(uint8_t addr) {
  _addr = addr;
  Wire.beginTransmission(_addr);
  Wire.write(MPU_PWR_MGMT_1);
  Wire.write(0x00);                         // reveil (sort du sleep)
  bool ok = (Wire.endTransmission() == 0);
  _health = ok ? HLTH_OK : ERR_I2C_NACK;
  _lastUs = micros();
  return ok;
}

bool Mpu6050::read() {
  Wire.beginTransmission(_addr);
  Wire.write(MPU_ACCEL_XOUT);
  if (Wire.endTransmission(false) != 0) { _health = ERR_I2C_NACK; return false; }
  if (Wire.requestFrom((int)_addr, 14) != 14) { _health = ERR_I2C_READ; return false; }
  _ax = (Wire.read() << 8) | Wire.read();
  _ay = (Wire.read() << 8) | Wire.read();
  _az = (Wire.read() << 8) | Wire.read();
  Wire.read(); Wire.read();                 // temperature (ignoree)
  _gx = (Wire.read() << 8) | Wire.read();
  _gy = (Wire.read() << 8) | Wire.read();
  _gz = (Wire.read() << 8) | Wire.read();
  _health = HLTH_OK;
  return true;
}

// Fusionne l'angle accel (absolu, bruite) et l'integration gyro (lisse, derive).
void Mpu6050::update() {
  if (!read()) return;
  uint32_t now = micros();
  float dt = (now - _lastUs) * 1e-6f;
  _lastUs = now;
  if (dt <= 0.0f || dt > 0.5f) return;       // premier tour / trou -> on saute

  float axf = _ax / ACC_LSB_PER_G;
  float ayf = _ay / ACC_LSB_PER_G;
  float azf = _az / ACC_LSB_PER_G;
  float rollAcc  = atan2(ayf, azf) * RAD2DEG;
  float pitchAcc = atan2(-axf, sqrt(ayf * ayf + azf * azf)) * RAD2DEG;

  float gxDps = _gx / GYR_LSB_PER_DPS;
  float gyDps = _gy / GYR_LSB_PER_DPS;
  _roll  = COMP_ALPHA * (_roll  + gxDps * dt) + (1.0f - COMP_ALPHA) * rollAcc;
  _pitch = COMP_ALPHA * (_pitch + gyDps * dt) + (1.0f - COMP_ALPHA) * pitchAcc;
}
