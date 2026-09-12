/*
 * Mpu6050.h - IMU MPU6050 (I2C) + filtre complementaire roll/pitch.
 *
 * Fusion a bord (98 % gyro / 2 % accel) pour un angle absolu peu bruite et sans
 * derive lente. Expose aussi les 6 valeurs brutes (accel/gyro) pour l'hote.
 * La sante remonte un code erreur (ERR_I2C_NACK / ERR_I2C_READ) en cas d'echec bus.
 */
#ifndef GROVEPI_MPU6050_H
#define GROVEPI_MPU6050_H

#include <Arduino.h>
#include "Protocol.h"   // codes ERR_*

class Mpu6050 {
public:
  bool    begin(uint8_t addr);          // reveil ; false si pas d'ACK
  void    update();                     // lit + fusionne (a appeler chaque tour)
  int16_t roll100() const  { return (int16_t)(_roll  * 100.0f); }
  int16_t pitch100() const { return (int16_t)(_pitch * 100.0f); }
  int16_t ax() const { return _ax; }  int16_t ay() const { return _ay; }
  int16_t az() const { return _az; }  int16_t gx() const { return _gx; }
  int16_t gy() const { return _gy; }  int16_t gz() const { return _gz; }
  uint8_t health() const { return _health; }   // HLTH_OK / ERR_I2C_*

private:
  bool    read();                       // lit 14 octets ; maj _health

  uint8_t  _addr    = 0x68;
  float    _roll    = 0.0f, _pitch = 0.0f;
  int16_t  _ax = 0, _ay = 0, _az = 0, _gx = 0, _gy = 0, _gz = 0;
  uint32_t _lastUs  = 0;
  uint8_t  _health  = HLTH_OK;
};

#endif // GROVEPI_MPU6050_H
