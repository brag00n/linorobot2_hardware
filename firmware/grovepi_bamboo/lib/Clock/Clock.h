/*
 * Clock.h - Horloge monotone interne (referentiel de temps de la carte).
 *
 * Compteur uint32 en MILLISECONDES pilote par une interruption materielle
 * DEDIEE (Timer2 en mode CTC @ 1 kHz), independante de millis() (Timer0 du core
 * Arduino). Objectif : un referentiel de temps qu'on maitrise, incremente par
 * l'ISR quoi qu'il arrive dans loop() (y compris pendant pulseIn, qui laisse les
 * interruptions actives).
 *
 * L'horloge est MONOTONE : elle ne recule jamais, ne saute jamais. La synchro
 * avec le temps exterieur (ROS) se fait cote hote (offset estime via TIME_SYNC),
 * pas en modifiant ce compteur -> indispensable pour la fusion multi-cartes.
 *
 * nowMs() lit le compteur 32 bits de facon ATOMIQUE (protege contre une lecture
 * dechiree pendant l'ISR).
 */
#ifndef GROVEPI_CLOCK_H
#define GROVEPI_CLOCK_H

#include <Arduino.h>

namespace Clock {
  void     begin();     // configure Timer2 CTC @ 1 kHz + active l'IRQ
  uint32_t nowMs();     // temps monotone en ms depuis begin() (lecture atomique)
}

#endif // GROVEPI_CLOCK_H
