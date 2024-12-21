#include <Arduino.h>

#ifndef _MOUVEMENT_H
#define _MOUVEMENT_H



void rotation(int consigne, int vitesse, int sens);
void ligne_droite(int consigne, int vitesse, int sens);
void recalage(int cons_distance_ticks, float SEUIL_RECALAGE, int vitesse_recalage, int sens_recalage);


#endif
