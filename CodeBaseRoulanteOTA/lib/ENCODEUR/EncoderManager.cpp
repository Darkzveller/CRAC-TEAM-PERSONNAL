#include "Variable.h" // Remonte d'un niveau pour atteindre lib
#include "EncoderManager.h"

ESP32Encoder encodergauche;
ESP32Encoder encoderdroite;

void setup_encodeur()
{
    ESP32Encoder::useInternalWeakPullResistors = UP; // Utilise les résistances internes
    encoderdroite.attachHalfQuad(tab_encodeur_droit[0], tab_encodeur_droit[1]);
    encodergauche.attachHalfQuad(tab_encodeur_gauche[0], tab_encodeur_gauche[1]);

    encoderdroite.clearCount();
    encodergauche.clearCount();
}

void read_encodeurdroit()
{

    long val_tick = encoderdroite.getCount()*COEFF_ROUE_DROITE;
    odo_droit = val_tick;
    
    float number_tour = val_tick * 1.0 / TIC_PER_TOUR;
    float angle_actuelle_radians = 2.0 * M_PI * number_tour / 1.0;
    theta_droit = angle_actuelle_radians;
    float distance_parcourue_translation = angle_actuelle_radians * SIZE_WHEEL_mm / (2.0 * M_PI);
    odo_dist_droit = distance_parcourue_translation;
    float vitesse_angulaire = (angle_actuelle_radians - angle_precedent_droit) / Te;
    angle_precedent_droit = angle_actuelle_radians;

    // Serial.printf("Tick= %.0f toru = %f angle %f vit %f dist %f\n", val_tick, number_tour, angle_actuelle_radians, vitesse_angulaire, distance_parcourue_translation);
    // if (grandeur_A_Mesure == SHOW_TICK)
    // {
    //     return val_tick;
    // }

    // if (grandeur_A_Mesure == SHOW_NUMBER_TOUR)
    // {
    //     return number_tour;
    // }
    // if (grandeur_A_Mesure == SHOW_ANGLE_RADIANS)
    // {
    //     return angle_actuelle_radians;
    // }
    // if (grandeur_A_Mesure == SHOW_DIST_MM)
    // {
    //     return distance_parcourue_translation;
    // }
    // if (grandeur_A_Mesure == SHOW_VITESSE_RAD_PAR_SEC)
    // {
    //     return vitesse_angulaire;
    // }
}

void read_encodeurgauche()
{

    long val_tick = encodergauche.getCount()*COEFF_ROUE_GAUCHE;
    odo_gauche = val_tick;

    float number_tour = val_tick * 1.0 / TIC_PER_TOUR;
    float angle_actuelle_radians = 2.0 * M_PI * number_tour / 1.0;
    theta_gauche = angle_actuelle_radians;
    float distance_parcourue_translation = angle_actuelle_radians * SIZE_WHEEL_mm / (2.0 * M_PI);
    odo_dist_gauche = distance_parcourue_translation;

    float vitesse_angulaire = (angle_actuelle_radians - angle_precedent_gauche) / Te;
    angle_precedent_gauche = angle_actuelle_radians;

    // Serial.printf("Tick= %.0f toru = %f angle %f vit %f dist %f\n", val_tick, number_tour, angle_actuelle_radians, vitesse_angulaire, distance_parcourue_translation);
    // if (grandeur_A_Mesure == SHOW_TICK)
    // {
    //     return val_tick;
    // }

    // if (grandeur_A_Mesure == SHOW_NUMBER_TOUR)
    // {
    //     return number_tour;
    // }
    // if (grandeur_A_Mesure == SHOW_ANGLE_RADIANS)
    // {
    //     return angle_actuelle_radians;
    // }
    // if (grandeur_A_Mesure == SHOW_DIST_MM)
    // {
    //     return distance_parcourue_translation;
    // }
    // if (grandeur_A_Mesure == SHOW_VITESSE_RAD_PAR_SEC)
    // {
    //     return vitesse_angulaire;
    // }
}
