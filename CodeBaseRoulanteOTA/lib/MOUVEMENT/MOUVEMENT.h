#include <Arduino.h>

#ifndef _MOUVEMENT_H
#define _MOUVEMENT_H

void rotation(int consigne, int vitesse);
void ligne_droite(int consigne, int vitesse);
double asservissement_angle_correction(double consigne_angle, double observation_angle);
void x_y_theta(float coordonnee_x, float coordonnee_y, float theta_fin, int vitesse);

#endif
