#include "Variable.h"
float tension_bat = 13;
// Définition des variables
// Encodeur
float perimetre_robot = M_PI * LARGEUR_ROBOT_mm;

double odo_dist_gauche;
double odo_dist_droit;

double theta_robot_prec = 0;
double theta_robot = 0;
double odo_x, odo_y;
double odo_last_d = 0;
double odo_last_g = 0;

double odo_tick_droit;
double odo_tick_gauche;
double odo_tick_droit_last;
double odo_tick_gauche_last;

double delta_odo_tick_droit;
double delta_odo_tick_gauche;

double delta_droit;
double delta_gauche;

double distance_parcourue;

double vitesse_rob = 0;
double vitesse_rob_roue_droite = 0;
double vitesse_rob_roue_gauche = 0;

float consigne_odo_droite_prec = 0;
float consigne_odo_gauche_prec = 0;
float consigne_theta_prec = 0;

double consigne_odo_x_prec = 0;
double consigne_odo_y_prec = 0;

//************Liste Ordre Deplacement */

Ordre_deplacement liste = {
    TYPE_DEPLACEMENT_IMMOBILE, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

// Variable asservissement
//************Asservissement ROUE FOLLE EN TICK */

// float coeff_P_roue_folle_tick_gauche = 8;
// float coeff_D_roue_folle_tick_gauche = 250;
// float coeff_I_roue_folle_tick_gauche = 0.0;

float coeff_P_roue_folle_tick_gauche = 10;
float coeff_D_roue_folle_tick_gauche = 0.25;
float coeff_I_roue_folle_tick_gauche = 0.4;

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
#ifdef PARAMETRE_BASE_OFFICIEL
float Dmax = 15;
#endif
#ifdef PARAMETRE_BASE_TEST
float Dmax = 5;
#endif
float limit_reprise_asser = 150;

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

float kp_vit = 0.25; // 2.5
float ki_vit = 0.0;  // 0.01
float kd_vit = 0.0;  // 0.05
float erreur_vit_precedente_roue_folle_droite = 0;
float integral_limit = 500;
float somme_erreur_vit_roue_folle_droite = 0;
float somme_erreur_vit_roue_folle_gauche = 0;
float erreur_vit_precedente_roue_folle_gauche = 0;

bool start_asservissement_roue_gauche = false;
bool start_asservissement_roue_droite = false;

Etat_vitesse_roue_folle_droite etat_actuel_vit_roue_folle_droite = ETAT_ATTENTE_Vitesse_ROUE_FOLLE_DROITE;
Etat_vitesse_roue_folle_gauche etat_actuel_vit_roue_folle_gauche = ETAT_ATTENTE_Vitesse_ROUE_FOLLE_GAUCHE;
// Fonction pour convertir un état en texte (roue folle gauche)
String toStringG(Etat_vitesse_roue_folle_gauche etat)
{
    switch (etat)
    {
    case ETAT_ATTENTE_Vitesse_ROUE_FOLLE_GAUCHE:
        return "ETAT_ATTENTE_Vitesse_ROUE_FOLLE_GAUCHE";
    case ETAT_ACCELERATION_Vitesse_ROUE_FOLLE_GAUCHE:
        return "ETAT_ACCELERATION_Vitesse_ROUE_FOLLE_GAUCHE";
    case ETAT_CROISIERE_Vitesse_ROUE_FOLLE_GAUCHE:
        return "ETAT_CROISIERE_Vitesse_ROUE_FOLLE_GAUCHE";
    case ETAT_DECELERATION_Vitesse_ROUE_FOLLE_GAUCHE:
        return "ETAT_DECELERATION_Vitesse_ROUE_FOLLE_GAUCHE";
    case ETAT_ARRET_Vitesse_ROUE_FOLLE_GAUCHE:
        return "ETAT_ARRET_Vitesse_ROUE_FOLLE_GAUCHE";
    case ETAT_VIDE_Vitesse_ROUE_FOLLE_GAUCHE:
        return "ETAT_VIDE_Vitesse_ROUE_FOLLE_GAUCHE";
    default:
        return "ETAT_INCONNU";
    }
}

// Fonction pour convertir un état en texte (roue folle droite)
String toStringD(Etat_vitesse_roue_folle_droite etat)
{
    switch (etat)
    {
    case ETAT_ATTENTE_Vitesse_ROUE_FOLLE_DROITE:
        return "ETAT_ATTENTE_Vitesse_ROUE_FOLLE_DROITE";
    case ETAT_ACCELERATION_Vitesse_ROUE_FOLLE_DROITE:
        return "ETAT_ACCELERATION_Vitesse_ROUE_FOLLE_DROITE";
    case ETAT_CROISIERE_Vitesse_ROUE_FOLLE_DROITE:
        return "ETAT_CROISIERE_Vitesse_ROUE_FOLLE_DROITE";
    case ETAT_DECELERATION_Vitesse_ROUE_FOLLE_DROITE:
        return "ETAT_DECELERATION_Vitesse_ROUE_FOLLE_DROITE";
    case ETAT_ARRET_Vitesse_ROUE_FOLLE_DROITE:
        return "ETAT_ARRET_Vitesse_ROUE_FOLLE_DROITE";
    case ETAT_VIDE_Vitesse_ROUE_FOLLE_DROITE:
        return "ETAT_VIDE_Vitesse_ROUE_FOLLE_DROITE";
    default:
        return "ETAT_INCONNU";
    }
}

//************************Consigne de vitesse */
float consigne_position_droite = 0;
float consigne_position_gauche = 0;
bool pause_asser_test = false;

//************************Asser Freinage */
float coeff_P_freinage = 10;
float coeff_D_freinage = 0;
float coeff_I_freinage = 0.0;
float integral_limit_freinage = 3075;
float somme_erreur_freinage_roue_folle_droite = 0;
float erreur_prec_freinage_roue_folle_droite = 0;
float somme_erreur_freinage_roue_folle_gauche = 0;
float erreur_prec_freinage_roue_folle_gauche = 0;
//************************Asser Correction d'angle */

double coeff_P_angle = -5;
double coeff_I_angle = 0;
double coeff_D_angle = 0;
double integral_limit_angle = 50.0;
// Variables globales pour le PID
double erreur_prec_angle = 0.0;    // Erreur précédente
double somme_integral_angle = 0.0; // Somme des erreurs pour le calcul intégral
double correction = 0;

//************************Mouvement */
int sens = 0;
int etat_x_y_theta = 0;

double theta_premiere_rotation = 0;
double theta_deuxieme_rotation = 0;
float consigne_regulation_moyenne = 0;

//************************Polaire en tick */

float erreur_distance = 0;
float erreur_orient = 0;
float consigne_dist_polaire_tick_max = SPEED_NORMAL;
float coeff_rot_polaire_tick = VALEUR_DEFAUT_COEFF_ROT_POLAIRE_TICK;
float coeff_dist_polaire_tick = VALEUR_DEFAUT_COEFF_DIST_POLAIRE_TICK;
float consigne_rot_polaire_tick = 0;
float consigne_dist_polaire_tick = 0;


//***********Loop******************* */

bool flag_controle = false;

//***********CAN******************* */

CANMessage rxMsg;
int etat_lecture_can;
unsigned char FIFO_ecriture = 0;
signed char FIFO_lecture = 0;
signed char FIFO_occupation = 0;
signed char FIFO_max_occupation = 0;
bool flag_can_printf = false;
//***********Ordre de déplacement******************* */
bool flag_fin_mvt_x_y_theta = true;
bool stop_start_robot_fin_match = false;

//*********** Qu'est ce qu'on voit au borne de la batterie******************* */
float courant = 0;
float tension = 0;
float puissance = 0;

//***********Detect obstacle******************* */

bool flag_stop_lidar = false;
