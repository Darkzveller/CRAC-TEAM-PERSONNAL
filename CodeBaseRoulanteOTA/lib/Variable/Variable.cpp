#include "Variable.h"

// Définition des variables
// Encodeur


double odo_dist_gauche;
double odo_dist_droit;

double theta_robot_prec = 0;
double theta_robot = 0;
double odo_x, odo_y;
double odo_last_d = 0;
double odo_last_g = 0;

double odo_tick_droit;
double odo_tick_gauche;

double delta_droit;
double delta_gauche;


// Variable asservissement
//************Asservissement ROUE FOLLE EN TICK */

float coeff_P_roue_folle_tick = 15;
float coeff_D_roue_folle_tick = 0.9;
float coeff_I_roue_folle_tick = 0.1;

float erreur_prec_roue_folle_droite_tick = 0;
float erreur_prec_roue_folle_gauche_tick = 0;
float integral_limit_roue_folle_tick = 500; 

float somme_integral_roue_folle_droite_tick = 0; 
float somme_integral_roue_folle_gauche_tick = 0;

