#include <Arduino.h>

#ifndef _MOUVEMENT_H
#define _MOUVEMENT_H



void rotation(int consigne, int vitesse, int sens);
void ligne_droite(int consigne, int vitesse, int sens);
void asservissement_correction_angle(double consigne, double observation);


#endif
