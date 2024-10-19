#include "Variable.h"

// Définition des variables
int frequence = 19500;
int resolution_pwm = 12;
// Moteur droit
int PWM_1 = 17;
int M1_INA = 26;
int M1_INB = 25;
int channel_1 = 0;
// Moteur gauche
int PWM_2 = 18;
int M2_INA = 16;
int M2_INB = 15;
int channel_2 = 1;
// Encodeur
uint8_t  tab_encodeur_droit[2] = {22, 23};
uint8_t  tab_encodeur_gauche[2] = {36, 39};

float angle_precedent_droit;
float angle_precedent_gauche;
float odo_gauche;
float odo_droit;

