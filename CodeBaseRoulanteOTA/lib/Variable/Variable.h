#include <Arduino.h>

#ifndef Variable_H
#define Variable_H
// Parametre FreeRTOS
#define Te 5

// Déclaration des variables externes
#define frequence 19500
#define resolution_pwm 12 
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

extern float theta_robot_prec;
extern float theta_robot ;
extern float odo_x, odo_y;
extern float odo_last_d ;
extern float odo_last_g ;
 
extern float delta_droit;
extern float delta_gauche;

extern uint8_t tab_encodeur_droit[2];
extern uint8_t tab_encodeur_gauche[2];

extern float odo_dist_gauche;
extern float odo_dist_droit;


#endif
