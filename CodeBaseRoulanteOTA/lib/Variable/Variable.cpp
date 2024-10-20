#include "Variable.h"

// Définition des variables
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
float Tau_odo = 100;

uint8_t  tab_encodeur_droit[2] = {23, 22};
uint8_t  tab_encodeur_gauche[2] = {36, 39};

float angle_precedent_droit;
float angle_precedent_gauche;

float odo_gauche;
float odo_droit;

float odo_dist_gauche;
float odo_dist_droit;

float theta_droit;
float theta_gauche;


// Variable asservissement

