#include "Variable.h"
#include "ASSERVISSEMENT.h"
#include "MOUVEMENT.h"
#include "MOTEUR.h"
#include <mat.h>
#include "USE_FUNCTION.h"

double somme_integral_correction_angle = 0;
double integral_limit_correction_angle = 500;
void rotation(int consigne, int vitesse, int sens)
{
    type_ligne_droite = false;
    // float vitesse_croisiere_droit = vitesse;
    // float vitesse_croisiere_gauche = vitesse;

    // int consigne_gauche = consigne - consigne_odo_gauche_prec;
    // int consigne_droite = consigne - consigne_odo_droite_prec;
    if (sens >= 1)
    {
        sens = 1;
    }
    else if (sens <= -1)
    {
        sens = -1;
    }
    float vitesse_croisiere_gauche = vitesse * sens;
    float vitesse_croisiere_droit = vitesse * -sens;

    int consigne_gauche = consigne * sens;
    int consigne_droite = consigne * -sens;
    consigne_gauche = (consigne_gauche + consigne_odo_gauche_prec);
    consigne_droite = (consigne_droite + consigne_odo_droite_prec);

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
    // Serial.printf(" consigne_gauche %d ", consigne_gauche);
    // Serial.printf(" consigne_droite %d ", consigne_droite);
    // Serial.printf(" consigne_odo_droite_prec %f ", consigne_odo_droite_prec);
    // Serial.printf(" consigne_odo_gauche_prec %f ", consigne_odo_gauche_prec);
    // Serial.println();

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
    // Serial.printf(" correction %f", correction);
    // Serial.printf(" erreur_angle_correction %f", erreur_angle_correction);

    // Serial.printf(" consigne_regulation_vitesse_droite %.2f ", consigne_regulation_vitesse_droite);
    // Serial.printf(" consigne_regulation_vitesse_gauche %.2f ", consigne_regulation_vitesse_gauche);
}

void x_y_theta(float coordonnee_x, float coordonnee_y, float theta_fin, int vitesse)
{

    float hypothenuse = sqrt(pow(coordonnee_x, 2) + pow(coordonnee_y, 2));
    // TOA : Tangente = Opposé / Adjacent.
    double theta_rotation = degrees(atan2(coordonnee_y, coordonnee_x));

    double theta_rotation_final = theta_fin - theta_rotation;
    Serial.printf(" etat_x_y_theta x %d ", etat_x_y_theta);

    // Serial.printf(" coordonnee_x %.3f ", coordonnee_x);
    // Serial.printf(" coordonnee_y %.3f ", coordonnee_y);
    // Serial.printf(" theta_fin %.3f ", theta_fin);
    // Serial.printf(" ce qui rentre dans atan %.3f ", atan2(coordonnee_y, coordonnee_x));
    // Serial.printf(" hypothenuse %.3f ", hypothenuse);
    // Serial.printf(" theta_rotation %.3f deg", theta_rotation);
    // Serial.printf(" theta_rotation_final %.3f deg", theta_rotation_final);
    int sens = 0;
    int time = 2000;
    switch (etat_x_y_theta)
    {
    case -1:
        break;
    case 0:
        if (theta_rotation > 0)
        {
            sens = 1;
        }
        if (theta_rotation < 0)
        {
            sens = -1;
        }

        rotation(fabs(convert_angle_deg_to_tick(theta_rotation)), vitesse, sens);

        if (return_flag_asser_roue())
        {

            stop_motors();
            etat_x_y_theta = 1;
            lauch_flag_asser_roue(true);
        }
        break;

    case 1:
        if (hypothenuse > 0)
        {
            sens = 1;
        }
        if (hypothenuse < 0)
        {
            sens = -1;
        }

        ligne_droite(fabs(convert_distance_mm_to_tick(hypothenuse)), vitesse, sens);

        if (return_flag_asser_roue())
        {
            stop_motors();

            etat_x_y_theta = 2;
            lauch_flag_asser_roue(true);
        }
        break;
    case 2:
        if (theta_rotation_final > 0)
        {
            sens = 1;
        }
        if (theta_rotation_final < 0)
        {
            sens = -1;
        }
        rotation(fabs(convert_angle_deg_to_tick(theta_rotation_final)), vitesse, sens);
        if (return_flag_asser_roue())
        {
            stop_motors();

            etat_x_y_theta = -1;
        }
        break;

    default:
        break;
    }
}