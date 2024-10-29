#include "Variable.h"

// Définition des variables
// Encodeur
double erreur_test;

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

double distance_parcourue;
// Variable asservissement
//************Asservissement ROUE FOLLE EN TICK */

float coeff_P_roue_folle_tick = 7/2.0;
float coeff_D_roue_folle_tick = 0.25/2.0;
float coeff_I_roue_folle_tick = 0.3*1.5;

float erreur_prec_roue_folle_droite_tick = 0;
float erreur_prec_roue_folle_gauche_tick = 0;
float integral_limit_roue_folle_tick = 500; 

float somme_integral_roue_folle_droite_tick = 0; 
float somme_integral_roue_folle_gauche_tick = 0;

//***********OTA******************* */

bool flag_controle = false;



