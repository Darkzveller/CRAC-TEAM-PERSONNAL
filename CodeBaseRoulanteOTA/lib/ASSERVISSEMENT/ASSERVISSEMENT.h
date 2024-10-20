#include <Arduino.h>

#ifndef _ASSERVISSEMENT_H
#define _ASSERVISSEMENT_H

// #define P
// #define PD
// #define PI
// #define PDI
// #define PID
// #define COEFF_P
// #define COEFF_PD
// #define COEFF_PI

void asservissement_roue_folle_droite_tick(double consigne, double observation);
void asservissement_roue_folle_gauche_tick(double consigne, double observation);
#endif
