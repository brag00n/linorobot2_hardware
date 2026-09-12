#include "SharpIR.h"

uint16_t SharpIR::read() {
  uint16_t v[SAMPLES];
  for (uint8_t i = 0; i < SAMPLES; i++) v[i] = analogRead(_pin);
  // Tri par insertion (SAMPLES petit) -> mediane = element central.
  for (uint8_t i = 1; i < SAMPLES; i++) {
    uint16_t x = v[i]; int8_t j = i - 1;
    while (j >= 0 && v[j] > x) { v[j + 1] = v[j]; j--; }
    v[j + 1] = x;
  }
  uint16_t adc = v[SAMPLES / 2];
  _adc = adc;

  // ADC colle a une borne -> capteur debranche / court-circuit.
  if (adc == 0)         _health = ERR_ADC_LOW;
  else if (adc >= 1023) _health = ERR_ADC_HIGH;
  else                  _health = HLTH_OK;

  uint16_t mv = (uint16_t)((uint32_t)adc * 5000UL / 1023UL);   // ADC -> millivolts
  if (mv < MV_MIN || mv > MV_MAX) {
    _last = DIST_NONE;                                         // hors plage utile
  } else {
    uint16_t cm = (uint16_t)(137500UL / (mv - 1125UL));
    _last = cm * 10;                                           // cm -> mm
  }
  return _last;
}
