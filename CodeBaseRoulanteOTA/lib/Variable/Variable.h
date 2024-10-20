#include <Arduino.h>

#ifndef Variable_H
#define Variable_H
// Parametre FreeRTOS
#define Te 5

// Déclaration des variables externes
#define frequence 19500
#define resolution_pwm 12 
// Moteur droit
extern int PWM_1;
extern int M1_INA;
extern int M1_INB;
extern int channel_1;
// Moteur Gauche
extern int PWM_2;
extern int M2_INA;
extern int M2_INB;
extern int channel_2;
// Encodeur + Parametre physique du robot
#define ENTRAXE 110.0
#define LARGEUR_ROBOT_mm 250.0
#define TIC_PER_TOUR 2048.0
#define RESOLUTION_ROUE_CODEUSE 10.0
#define COEFF_ROUE_DROITE 1
#define COEFF_ROUE_GAUCHE 1
#define SIZE_WHEEL_DIAMETER_mm 50.0
#define SHOW_TICK 0
#define SHOW_NUMBER_TOUR 1
#define SHOW_ANGLE_RADIANS 2
#define SHOW_DIST_MM 3
#define SHOW_VITESSE_RAD_PAR_SEC 4


extern float theta_robot_prec;
extern float theta_robot ;
extern float odo_x, odo_y;
extern float odo_last_d ;
extern float odo_last_g ;
 
extern float delta_droit;
extern float delta_gauche;


extern uint8_t tab_encodeur_droit[2];
extern uint8_t tab_encodeur_gauche[2];

extern float Tau_odo;

extern float angle_precedent_droit;
extern float angle_precedent_gauche;

extern float odo_tick_gauche;
extern float odo_tick_droit;

extern float odo_dist_gauche;
extern float odo_dist_droit;

extern float theta_droit;
extern float theta_gauche;

#endif
