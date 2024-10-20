#include <Arduino.h>

#ifndef Variable_H
#define Variable_H
// Parametre FreeRTOS
#define Te 5

// Déclaration des variables externes
#define frequence 19500
#define resolution_pwm 12
#define POURCENT_MAX_PWM 0.75 
// Moteur droit
#define PWM_1 17 
#define M1_INA 26
#define M1_INB 25
#define channel_1 0
// Moteur Gauche
#define PWM_2 18
#define M2_INA 16
#define M2_INB 15
#define channel_2 1
// Encodeur + Parametre physique du robot
#define ENTRAXE 110.0
#define LARGEUR_ROBOT_mm 250.0
#define TIC_PER_TOUR 2048.0
#define RESOLUTION_ROUE_CODEUSE 10.0
#define COEFF_ROUE_DROITE 1
#define COEFF_ROUE_GAUCHE 1
#define SIZE_WHEEL_DIAMETER_mm 50.0

extern double theta_robot_prec;
extern double theta_robot ;

extern double odo_x, odo_y;
extern double odo_last_d ;
extern double odo_last_g ;
 
extern double odo_tick_droit;
extern double odo_tick_gauche;

extern double delta_droit;
extern double delta_gauche;

#define PIN_ENCODEUR_1 23
#define PIN_ENCODEUR_2 22

#define PIN_ENCODEUR_3 36
#define PIN_ENCODEUR_4 39
// uint8_t  tab_encodeur_droit[2] = {23, 22};
// uint8_t  tab_encodeur_gauche[2] = {36, 39};

extern double odo_dist_gauche;
extern double odo_dist_droit;
//************Asservissement ROUE FOLLE EN TICK */

extern float coeff_P_roue_folle_tick ;
extern float coeff_D_roue_folle_tick ;
extern float coeff_I_roue_folle_tick ;

extern float erreur_prec_roue_folle_droite_tick;
extern float erreur_prec_roue_folle_gauche_tick ;
extern float integral_limit_roue_folle_tick ; 

extern float somme_integral_roue_folle_droite_tick; 
extern float somme_integral_roue_folle_gauche_tick; 



//***********OTA******************* */

extern bool flag_controle ;


#endif
