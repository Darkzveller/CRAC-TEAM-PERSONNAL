#include <Arduino.h>

#ifndef _USE_FUNCTION_H
#define _USE_FUNCTION_H

int16_t fusion_octet(int octet0, int octet1);
int convert_angle_deg_to_tick(float angle);
float convert_angle_radian_to_tick(float angle);
int convert_distance_mm_to_tick(float distance);
void pourcentage_erreur(float val_theorique, float valeur_experimentale);
#endif


