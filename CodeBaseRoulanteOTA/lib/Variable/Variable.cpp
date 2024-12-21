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

double distance_parcourue;

float consigne_odo_droite_prec = 0;
float consigne_odo_gauche_prec = 0;
float consigne_theta_prec = 0;

// Variable asservissement
//************Asservissement ROUE FOLLE EN TICK */

float coeff_P_roue_folle_tick_gauche = 7.0 / 2;
float coeff_D_roue_folle_tick_gauche = 0.25 / 2;
float coeff_I_roue_folle_tick_gauche = 0.3 * 2;

float coeff_P_roue_folle_tick_droite = coeff_P_roue_folle_tick_gauche;
float coeff_D_roue_folle_tick_droite = coeff_D_roue_folle_tick_gauche;
float coeff_I_roue_folle_tick_droite = coeff_I_roue_folle_tick_gauche;

float erreur_prec_roue_folle_droite_tick = 0;
float erreur_prec_roue_folle_gauche_tick = 0;
float integral_limit_roue_folle_tick = 500;

float somme_integral_roue_folle_droite_tick = 0;
float somme_integral_roue_folle_gauche_tick = 0;

//************************Asservissement vitesse Roue folle en TICK */

float Vmax = 200;
float Amax = 50;
// float Dmax = 2.5;
float Dmax = 7;
float limit_reprise_asser = 100;

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
double Tc_counter_droite = 0;
double Tc_counter_gauche = 0;
double T_attente_gauche = 10;
double T_counter_attente_gauche;
double T_attente_droite = T_attente_gauche;
double T_counter_attente_droite;

volatile bool type_ligne_droite = false;

float distance_accel_droite = 0;
float distance_decl_droite = 0;
float distance_accel_gauche = 0;
float distance_decl_gauche = 0;

float kp_vit = 2.5;
float ki_vit = 0.01;
float kd_vit = 0.05;
float erreur_vit_precedente_roue_folle_droite = 0;
float integral_limit = 500;
float somme_erreur_vit_roue_folle_droite = 0;
float somme_erreur_vit_roue_folle_gauche = 0;
float erreur_vit_precedente_roue_folle_gauche = 0;

bool start_asservissement_roue_gauche = false;
bool start_asservissement_roue_droite = false;

Etat_vitesse_roue_folle_droite etat_actuel_vit_roue_folle_droite = ETAT_ATTENTE_Vitesse_ROUE_FOLLE_DROITE;
Etat_vitesse_roue_folle_gauche etat_actuel_vit_roue_folle_gauche = ETAT_ATTENTE_Vitesse_ROUE_FOLLE_GAUCHE;

//************************Convitesse de vitesse */
float consigne_regulation_vitesse_droite = 0;
float consigne_regulation_vitesse_gauche = 0;

//***********OTA******************* */

bool flag_controle = false;

//***********CAN******************* */

CANMessage myData;     // data received by BT to write on CAN
CANMessage DATAtoSend; // data received by CAN to send on BT

CANMessage rxMsg[SIZE_FIFO]; // data received by CAN to control the robot
CANMessage DATArobot;        // DATA that the robot will write on CAN
unsigned char FIFO_ecriture = 0;
signed char FIFO_lecture = 0;
signed char FIFO_occupation = 0;
signed char FIFO_max_occupation = 0;
//***********Ordre de déplacement******************* */
bool flag_fin_mvt = true;
