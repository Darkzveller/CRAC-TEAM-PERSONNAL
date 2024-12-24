#include "Variable.h"
#include "ASSERVISSEMENT.h"
#include "MOUVEMENT.h"
#include "MOTEUR.h"
double somme_integral_correction_angle = 0;
double integral_limit_correction_angle = 500;
void rotation(int consigne, int vitesse, int sens)
{
    type_ligne_droite = false;
    // float vitesse_croisiere_droit = vitesse;
    // float vitesse_croisiere_gauche = vitesse;

    // int consigne_gauche = consigne - consigne_odo_gauche_prec;
    // int consigne_droite = consigne - consigne_odo_droite_prec;

    float vitesse_croisiere_gauche = vitesse * sens;
    float vitesse_croisiere_droit = vitesse * -sens;

    int consigne_gauche = consigne * sens;
    int consigne_droite = consigne * -sens;
    consigne_gauche = (consigne_gauche + consigne_odo_gauche_prec);
    consigne_droite = (consigne_droite + consigne_odo_droite_prec);
    // Serial.printf(" consigne_gauche %d ", consigne_gauche);
    // Serial.printf(" consigne_droite %d ", consigne_droite);

    // Serial.printf(" vitesse_croisiere_gauche %.0f ", vitesse_croisiere_gauche);
    // Serial.printf(" vitesse_croisiere_droit %.0f ", vitesse_croisiere_droit);
    // stop_motors();

    // Serial.println();
    // Serial.println();
    // Serial.println();
    // while (true)
    // {
    //     Serial.printf(" consigne_gauche %d ", consigne_gauche);
    //     Serial.printf(" consigne_droite %d ", consigne_droite);

    //     Serial.printf(" consigne_odo_gauche_prec %.0f ", consigne_odo_gauche_prec);
    //     Serial.printf(" consigne_odo_droite_prec %.0f ", consigne_odo_droite_prec);
    //     Serial.printf("| odo gauche %.0f odo droite %.0f", odo_tick_gauche, odo_tick_droit);

    //     Serial.println();
    // }
    // Serial.println();

    // Calcul initial des consignes de vitesse pour chaque roue
    consigne_regulation_vitesse_droite = regulation_vitesse_roue_folle_droite(consigne_droite, vitesse_croisiere_droit);
    consigne_regulation_vitesse_gauche = regulation_vitesse_roue_folle_gauche(consigne_gauche, vitesse_croisiere_gauche);

    // Imposer une symétrie des consignes de vitesse
    float vitesse_moyenne = (consigne_regulation_vitesse_droite + consigne_regulation_vitesse_gauche) / 2;

    // On force les consignes à être égales et opposées
    // consigne_regulation_vitesse_droite = -sens * vitesse_moyenne;
    // consigne_regulation_vitesse_gauche = sens * vitesse_moyenne;    Serial.printf(" vitesse_moyenne %.0f ", vitesse_moyenne);

    // Serial.printf(" consigne_regulation_vitesse_droite %.0f ", consigne_regulation_vitesse_droite);
    // Serial.printf(" consigne_regulation_vitesse_gauche %.0f ", consigne_regulation_vitesse_gauche);
}
void ligne_droite(int consigne, int vitesse, int sens)
{
    type_ligne_droite = 0;
    float vitesse_croisiere_gauche = vitesse * sens;
    float vitesse_croisiere_droit = vitesse * sens;

    int consigne_gauche = consigne * sens;
    int consigne_droite = consigne * sens;
    consigne_gauche = (consigne_gauche + consigne_odo_gauche_prec);
    consigne_droite = (consigne_droite + consigne_odo_droite_prec);
    // stop_motors();

    // Serial.println();
    // Serial.println();
    // Serial.println();
    // while (true)
    // {
    //     Serial.printf(" consigne_gauche %d ", consigne_gauche);
    //     Serial.printf(" consigne_droite %d ", consigne_droite);

    //     Serial.printf(" consigne_odo_gauche_prec %.0f ", consigne_odo_gauche_prec);
    //     Serial.printf(" consigne_odo_droite_prec %.0f ", consigne_odo_droite_prec);
    //     Serial.printf("| odo gauche %.0f odo droite %.0f", odo_tick_gauche, odo_tick_droit);

    //     Serial.println();
    // }
    // Serial.println();

    // delay(1000000);
    // Calcul initial des consignes de vitesse pour chaque roue
    consigne_regulation_vitesse_droite = regulation_vitesse_roue_folle_droite(consigne_droite, vitesse_croisiere_droit);
    consigne_regulation_vitesse_gauche = regulation_vitesse_roue_folle_gauche(consigne_gauche, vitesse_croisiere_gauche);
    float kp_angle_correction = 0.5;
    double coeff_I_correction_angle = 0.0014;

    float erreur_angle_correction = consigne_regulation_vitesse_droite - consigne_regulation_vitesse_gauche;
    somme_integral_correction_angle += erreur_angle_correction * Te;
    if (somme_integral_correction_angle > integral_limit_correction_angle)
    {
        somme_integral_correction_angle = integral_limit_correction_angle;
    }
    else if (somme_integral_correction_angle < -integral_limit_correction_angle)
    {
        somme_integral_correction_angle = -integral_limit_correction_angle;
    }
    float correction = kp_angle_correction * erreur_angle_correction + coeff_I_correction_angle * somme_integral_correction_angle;

    consigne_regulation_vitesse_droite -= correction;
    consigne_regulation_vitesse_gauche += correction;

    // asservissement_correction_angle(0, degrees(theta_robot));
    Serial.printf(" correction %f", correction);
    Serial.printf(" erreur_angle_correction %f", erreur_angle_correction);

    Serial.printf(" consigne_regulation_vitesse_droite %.2f ", consigne_regulation_vitesse_droite);
    Serial.printf(" consigne_regulation_vitesse_gauche %.2f ", consigne_regulation_vitesse_gauche);
}
