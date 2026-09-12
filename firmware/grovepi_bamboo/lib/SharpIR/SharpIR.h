/*
 * SharpIR.h - Telemetre IR analogique Sharp GP2Y0A710K0F (100-550 cm).
 *
 * Sortie Vo NON LINEAIRE sur une entree analogique. Conversion issue de la lib
 * guillaume-rico/SharpIR (modele 100550) :
 *   current_mV  = map(adc, 0..1023, 0..5000)
 *   valide seulement si current_mV dans [1400, 3300]  (~63..500 cm)
 *   distance_cm = 137500 / (current_mV - 1125)
 * On lit une mediane de N echantillons (robuste aux pics) et on conserve l'ADC
 * brut pour la calibration. health() flague un ADC bloque a 0/1023.
 */
#ifndef GROVEPI_SHARPIR_H
#define GROVEPI_SHARPIR_H

#include <Arduino.h>
#include "Protocol.h"

class SharpIR {
public:
  void     begin(uint8_t pin) { _pin = pin; }
  uint16_t read();                            // mediane -> mm (DIST_NONE si hors plage)
  uint16_t last() const   { return _last; }
  uint16_t rawAdc() const { return _adc; }
  uint8_t  pin() const    { return _pin; }
  uint8_t  health() const { return _health; } // HLTH_OK / ERR_ADC_LOW / ERR_ADC_HIGH

private:
  static const uint8_t  SAMPLES = 9;          // impair -> mediane franche
  static const uint16_t MV_MIN  = 1400;       // borne basse de validite (~500 cm)
  static const uint16_t MV_MAX  = 3300;       // borne haute de validite (~63 cm)

  uint8_t  _pin    = A0;
  uint16_t _last   = DIST_NONE;
  uint16_t _adc    = 0;
  uint8_t  _health = HLTH_OK;
};

#endif // GROVEPI_SHARPIR_H
