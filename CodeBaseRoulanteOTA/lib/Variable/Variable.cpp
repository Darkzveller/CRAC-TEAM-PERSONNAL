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

float coeff_P_roue_folle_tick = 7.0;
float coeff_D_roue_folle_tick = 0.25;
float coeff_I_roue_folle_tick = 0.3;

float erreur_prec_roue_folle_droite_tick = 0;
float erreur_prec_roue_folle_gauche_tick = 0;
float integral_limit_roue_folle_tick = 500; 

float somme_integral_roue_folle_droite_tick = 0; 
float somme_integral_roue_folle_gauche_tick = 0;

//************************Asservissement vitesse Roue folle en TICK */

float Vmax = 145;
float Amax = 50;
float Dmax = 7.5 * 1.5;

float acc_actuel_droite = 0;
double consigne_vit_droite = 0;
double consigne_dist_droite = 0;

float acc_actuel_gauche = 0;
double consigne_vit_gauche = 0;
double consigne_dist_gauche = 0;

double Ta_counter_droite = 0;
double Ta_counter_gauche = 0;
double Td_counter_droite = 0;
double Td_counter_gauche = 0;

float distance_accel_droite = 0;
float distance_decl_droite = 0;
float distance_accel_gauche = 0;
float distance_decl_gauche = 0;

float kp_vit = 2.5;
float ki_vit = 0.1;
float kd_vit = 0.05;
float erreur_vit_precedente_roue_folle_droite = 0;
float integral_limit = 500;
float somme_erreur_vit_roue_folle_droite = 0;
float somme_erreur_vit_roue_folle_gauche = 0;
float erreur_vit_precedente_roue_folle_gauche = 0;

Etat_vitesse_roue_folle_droite etat_actuel_vit_roue_folle_droite = ETAT_ACCELERATION_Vitesse_ROUE_FOLLE_DROITE;
Etat_vitesse_roue_folle_gauche etat_actuel_vit_roue_folle_gauche = ETAT_ACCELERATION_Vitesse_ROUE_FOLLE_GAUCHE;

//***********OTA******************* */

bool flag_controle = false;



