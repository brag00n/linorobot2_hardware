#include "Ultrasonic.h"

static const uint16_t MAX_RANGE_CM    = 250;    // portee plafonnee
static const uint32_t ECHO_TIMEOUT_US = (uint32_t)MAX_RANGE_CM * 58UL + 400UL;
static const uint8_t  MISS_FOR_ERROR  = 5;      // echecs avant de flaguer ERR_NO_ECHO

void Ultrasonic::setPins(uint8_t trig, uint8_t echo) {
  _trig = trig;
  _echo = echo;
  pinMode(_trig, OUTPUT);
  digitalWrite(_trig, LOW);
  pinMode(_echo, INPUT);
}

uint16_t Ultrasonic::ping() {
  digitalWrite(_trig, LOW);
  delayMicroseconds(3);
  digitalWrite(_trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(_trig, LOW);

  uint32_t dur = pulseIn(_echo, HIGH, ECHO_TIMEOUT_US);
  if (dur == 0) {
    _last = DIST_NONE;                        // pas d'echo (hors portee / absent)
    if (_miss < MISS_FOR_ERROR) _miss++;
    if (_miss >= MISS_FOR_ERROR) _health = ERR_NO_ECHO;
  } else {
    uint32_t mm = (dur * 10UL) / 58UL;        // aller-retour : 58 us/cm
    _last = (mm > 65534UL) ? 65534 : (uint16_t)mm;
    _miss = 0;
    _health = HLTH_OK;
  }
  return _last;
}
