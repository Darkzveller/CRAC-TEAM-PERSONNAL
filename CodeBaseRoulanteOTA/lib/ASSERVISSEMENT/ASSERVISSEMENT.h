#include <Arduino.h>

#ifndef _ASSERVISSEMENT_H
#define _ASSERVISSEMENT_H



void asservissement_roue_folle_droite_tick(double consigne, double observation);
void asservissement_roue_folle_gauche_tick(double consigne, double observation);

double regulation_vitesse_roue_folle_droite(float cons, float Vmax_consigne);
double regulation_vitesse_roue_folle_gauche(float cons, float Vmax_consigne);

void asservissement_freinage_roue_folle_gauche(float consigne, float observation);

void asservissement_freinage_roue_folle_droite(float consigne, float observation);

#endif
