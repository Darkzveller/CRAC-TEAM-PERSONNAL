#include <Arduino.h>

#ifndef _MOTEUR_H
#define _MOTEUR_H

void setup_motors();
void stop_motors();

void moteur_droit(int pwm, bool sens);
void moteur_gauche(int pwm, bool sens);
#endif
