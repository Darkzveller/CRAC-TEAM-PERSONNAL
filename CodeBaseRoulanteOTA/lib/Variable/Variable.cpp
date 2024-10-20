#include "Variable.h"

// Définition des variables
// Encodeur

uint8_t  tab_encodeur_droit[2] = {23, 22};
uint8_t  tab_encodeur_gauche[2] = {36, 39};

float odo_dist_gauche;
float odo_dist_droit;


float theta_robot_prec = 0;
float theta_robot = 0;
float odo_x, odo_y;
float odo_last_d = 0;
float odo_last_g = 0;

float delta_droit;
float delta_gauche;


// Variable asservissement

