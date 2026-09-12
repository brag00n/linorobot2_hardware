/*
 * Ultrasonic.h - Telemetre HC-SR04 (un canal), lecture bloquante pulseIn().
 *
 * Un objet = un capteur (Trig/Echo). ping() lance une mesure et renvoie la
 * distance en mm (DIST_NONE si pas d'echo). health() remonte ERR_NO_ECHO apres
 * plusieurs echecs consecutifs (capteur absent / hors portee persistante).
 */
#ifndef GROVEPI_ULTRASONIC_H
#define GROVEPI_ULTRASONIC_H

#include <Arduino.h>
#include "Protocol.h"

class Ultrasonic {
public:
  void     setPins(uint8_t trig, uint8_t echo);  // (re)configure les broches
  uint16_t ping();                                // mesure -> mm (DIST_NONE si rien)
  uint16_t last() const   { return _last; }
  uint8_t  trig() const   { return _trig; }
  uint8_t  echo() const   { return _echo; }
  uint8_t  health() const { return _health; }     // HLTH_OK / ERR_NO_ECHO

private:
  uint8_t  _trig = 0xFF, _echo = 0xFF;
  uint16_t _last = DIST_NONE;
  uint8_t  _miss = 0;                              // echecs consecutifs
  uint8_t  _health = HLTH_OK;
};

#endif // GROVEPI_ULTRASONIC_H
