#include "Variable.h"
#include "ASSERVISSEMENT.h"
#include "MOUVEMENT.h"
#include "MOTEUR.h"
#include <mat.h>
#include "USE_FUNCTION.h"
void rotation(int consigne, int vitesse, int sens)
{
    type_ligne_droite = false;
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
    int difference_roue_odo = consigne_odo_gauche_prec + consigne_odo_droite_prec;

    consigne_gauche = (consigne_gauche + consigne_odo_gauche_prec);
    consigne_droite = (consigne_droite + consigne_odo_droite_prec - difference_roue_odo);
    // Serial.printf(" difference_roue_odo %d ", difference_roue_odo);
    // Serial.printf(" consigne_odo_gauche_prec %.0f ", consigne_odo_gauche_prec);
    // Serial.printf(" consigne_odo_droite_prec %.0f \n", consigne_odo_droite_prec);

    // Calcul initial des consignes de vitesse pour chaque roue
    consigne_regulation_vitesse_droite = regulation_vitesse_roue_folle_droite(consigne_droite, vitesse_croisiere_droit);
    consigne_regulation_vitesse_gauche = regulation_vitesse_roue_folle_gauche(consigne_gauche, vitesse_croisiere_gauche);

    // if (fabs(consigne_regulation_vitesse_droite) > fabs(consigne_regulation_vitesse_gauche))
    // {
    //     if (consigne_dist_droite > 0)
    //     {
    //         consigne_regulation_vitesse_droite = fabs(consigne_regulation_vitesse_gauche);
    //     }
    //     else
    //     {
    //         consigne_regulation_vitesse_droite = -fabs(consigne_regulation_vitesse_gauche);
    //     }
    // }
    // else if (fabs(consigne_regulation_vitesse_gauche) > fabs(consigne_regulation_vitesse_droite))
    // {
    //     if (consigne_dist_gauche > 0)
    //     {
    //         consigne_regulation_vitesse_gauche = fabs(consigne_regulation_vitesse_droite);
    //     }
    //     else
    //     {
    //         consigne_regulation_vitesse_gauche = -fabs(consigne_regulation_vitesse_droite);
    //     }
    // }
    // // Imposer une symétrie des consignes de vitesse
    // vitesse_moyenne = (fabs(consigne_regulation_vitesse_droite) + fabs(consigne_regulation_vitesse_gauche)) / 2;

    // // On force les consignes à être égales et opposées
    // consigne_regulation_vitesse_droite = -sens * vitesse_moyenne;
    // consigne_regulation_vitesse_gauche = sens * vitesse_moyenne;
    // Serial.printf(" consigne_regulation_vitesse_droite %.0f ", consigne_regulation_vitesse_droite);
    // Serial.printf(" consigne_regulation_vitesse_gauche %.0f ", consigne_regulation_vitesse_gauche);

    // float kp_angle_correction =1;

    // float erreur_angle_correction = consigne_regulation_vitesse_gauche + consigne_regulation_vitesse_droite;
    // float correction = kp_angle_correction * erreur_angle_correction;
    // // consigne_regulation_vitesse_droite -= correction;
    // consigne_regulation_vitesse_gauche -= correction;
    // Serial.printf(" correction %f ", correction);
    // Serial.printf(" erreur_angle_correction %f ", erreur_angle_correction);

    // Serial.printf(" consigne_regulation_vitesse_droite8CC %.0f ", consigne_regulation_vitesse_droite);
    // Serial.printf(" consigne_regulation_vitesse_gauche8CC %.0f ", consigne_regulation_vitesse_gauche);
    // Serial.printf(" teheta %.3f ", degrees(theta_robot));

    // Serial.println();
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
    // Correction d'angle basée sur l'écart entre les consignes gauche et droite
    double consigne_angle = consigne_theta_prec;     // Consigne d'angle cible (par exemple, maintenir 0°)
    double observation_angle = degrees(theta_robot); // Angle actuel du robot
    double correction = asservissement_angle_correction(consigne_angle, observation_angle);

    // Appliquer la correction à la consigne de vitesse
    consigne_regulation_vitesse_droite -= correction;
    consigne_regulation_vitesse_gauche += correction;
}

void x_y_theta(float coordonnee_x, float coordonnee_y, float theta_fin, int vitesse)
{

    float hypothenuse = sqrt(pow(coordonnee_x, 2) + pow(coordonnee_y, 2));
    // TOA : Tangente = Opposé / Adjacent.
    theta_premiere_rotation = degrees(atan2(coordonnee_y, coordonnee_x));

    theta_deuxieme_rotation = theta_fin - theta_premiere_rotation;
    // Serial.printf(" Odo x %.3f ", odo_x);
    // Serial.printf(" odo_y %.3f ", odo_y);
    // Serial.printf(" teheta %.3f ", degrees(theta_robot));
    // Serial.printf(" etat_x_y_theta x %d ", etat_x_y_theta);

    // Serial.printf(" coordonnee_x %.3f ", coordonnee_x);
    // Serial.printf(" coordonnee_y %.3f ", coordonnee_y);
    // Serial.printf(" theta_fin %.3f ", theta_fin);
    // Serial.print("Etat actuel : " + toStringG(etat_actuel_vit_roue_folle_gauche));
    // Serial.println(" " + toStringD(etat_actuel_vit_roue_folle_droite));

    // Serial.printf(" ce qui rentre dans atan %.3f ", atan2(coordonnee_y, coordonnee_x));
    // Serial.printf(" hypothenuse %.3f ", hypothenuse);
    // Serial.printf(" theta_premiere_rotation %.3f deg", theta_premiere_rotation);
    // Serial.printf(" theta_rotation_final %.3f deg", theta_rotation_final);
    // Serial.println();
    int sens = 0;
    int time = 100;
    switch (etat_x_y_theta)
    {
    case -1:
        break;
    case 0:
        if (theta_premiere_rotation >= 0)
        {
            sens = 1;
        }
        else if (theta_premiere_rotation < 0)
        {
            sens = -1;
        }

        rotation(convert_angle_deg_to_tick(fabs(theta_premiere_rotation)), vitesse, sens);

        if (return_flag_asser_roue())
        {

            stop_motors();
            delay(time);
            etat_x_y_theta = 1;
            lauch_flag_asser_roue(true);
            consigne_theta_prec = degrees(theta_robot);
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
            delay(time);
            etat_x_y_theta = 2;
            lauch_flag_asser_roue(true);
        }
        break;
    case 2:
        if (theta_deuxieme_rotation > 0)
        {
            sens = 1;
        }
        if (theta_deuxieme_rotation < 0)
        {
            sens = -1;
        }
        rotation(fabs(convert_angle_deg_to_tick(theta_deuxieme_rotation)), vitesse, sens);
        if (return_flag_asser_roue())
        {
            stop_motors();
            delay(time);
            consigne_theta_prec = degrees(theta_robot);

            etat_x_y_theta = -1;
        }
        break;

    default:
        break;
    }
}
double coeff_P_angle = 1.0;          // Coefficient proportionnel
double coeff_I_angle = 0.5;          // Coefficient intégral
double coeff_D_angle = 0.4;          // Coefficient dérivé
double integral_limit_angle = 100.0; // Limite de la somme intégrale pour éviter le dépassement
// Variables globales pour le PID
double erreur_prec_angle = 0.0;    // Erreur précédente
double somme_integral_angle = 0.0; // Somme des erreurs pour le calcul intégral
double asservissement_angle_correction(double consigne_angle, double observation_angle)
{
    static double erreur_prec_angle = 0.0;
    static double somme_integral_angle = 0.0;
    double erreur = consigne_angle - observation_angle;

    double proportionnel = coeff_P_angle * erreur;

    // Terme dérivé
    double deriver = coeff_D_angle * (erreur - erreur_prec_angle) / Te;

    // Terme intégral avec anti-windup
    somme_integral_angle += erreur * Te;
    if (somme_integral_angle > integral_limit_angle)
    {
        somme_integral_angle = integral_limit_angle;
    }
    else if (somme_integral_angle < -integral_limit_angle)
    {
        somme_integral_angle = -integral_limit_angle;
    }
    double integral = coeff_I_angle * somme_integral_angle;

    // Calcul de la correction
    double correction = proportionnel + deriver + integral;

    // Mise à jour de l'erreur précédente
    erreur_prec_angle = erreur;

    return correction; // Retourne la correction à appliquer
}
