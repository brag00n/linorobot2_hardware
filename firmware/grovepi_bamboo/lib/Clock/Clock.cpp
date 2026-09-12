#include "Clock.h"
#include <avr/interrupt.h>

// Compteur de millisecondes, incremente uniquement par l'ISR Timer2.
static volatile uint32_t g_ms = 0;

// Timer2 en CTC : 16 MHz / 64 = 250 kHz ; OCR2A = 249 -> 250 ticks = 1,000 ms pile.
ISR(TIMER2_COMPA_vect) {
  g_ms++;
}

namespace Clock {

void begin() {
  uint8_t s = SREG;
  cli();
  TCCR2A = _BV(WGM21);              // mode CTC (TOP = OCR2A)
  TCCR2B = _BV(CS22);               // prescaler 64
  OCR2A  = 249;                     // (250 kHz / 250) = 1 kHz -> IRQ chaque ms
  TCNT2  = 0;
  TIMSK2 = _BV(OCIE2A);             // active l'interruption de comparaison A
  g_ms   = 0;
  SREG = s;
}

uint32_t nowMs() {
  uint8_t s = SREG;
  cli();                            // lecture atomique du compteur 32 bits
  uint32_t m = g_ms;
  SREG = s;
  return m;
}

} // namespace Clock
