#include <Arduino.h>

#ifndef _MOTEUR_H
#define _MOTEUR_H

void setup_motors();
void stop_motors();

void moteur_droit(int pwm, bool sens);
void moteur_gauche(int pwm, bool sens);

void stop_moteur_droit();
void stop_moteur_gauche();

void freinage_moteur_gauche(bool on_off, int Vmax_consigne);
void freinage_moteur_droit(bool on_off, int Vmax_consigne);

#endif
