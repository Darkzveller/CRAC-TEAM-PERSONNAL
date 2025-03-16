#include "Variable.h"
#include "ASSERVISSEMENT.h"
#include "MOUVEMENT.h"
#include "MOTEUR.h"
#include <mat.h>
#include "USE_FUNCTION.h"
void rotation(int consigne, int vitesse)
{
    if (consigne >= 0)
    {
        sens = 1;
    }
    else
    {
        sens = -1;
    }
    float vitesse_croisiere_gauche = vitesse * sens;
    float vitesse_croisiere_droit = vitesse * -sens;

    int consigne_gauche = consigne;
    int consigne_droite = consigne * -1.0;
    consigne_gauche = (consigne_gauche + consigne_odo_gauche_prec);
    consigne_droite = (consigne_droite + consigne_odo_droite_prec);

    consigne_regulation_vitesse_droite = regulation_vitesse_roue_folle_droite(consigne_droite, vitesse_croisiere_droit);
    consigne_regulation_vitesse_gauche = regulation_vitesse_roue_folle_gauche(consigne_gauche, vitesse_croisiere_gauche);
    if ((etat_actuel_vit_roue_folle_gauche != ETAT_DECELERATION_Vitesse_ROUE_FOLLE_GAUCHE) || (etat_actuel_vit_roue_folle_droite != ETAT_DECELERATION_Vitesse_ROUE_FOLLE_DROITE))
    {
        float ecart = consigne_odo_gauche_prec + consigne_odo_droite_prec;
        consigne_regulation_vitesse_gauche = -consigne_regulation_vitesse_droite + ecart;
    }
    // // // On force les consignes à être égales et opposées
    // consigne_regulation_vitesse_droite = sens * consigne_regulation_moyenne;
    // consigne_regulation_vitesse_gauche = -sens * consigne_regulation_moyenne;
}

void ligne_droite(int consigne, int vitesse)
{
    if (consigne >= 0)
    {
        sens = 1;
    }
    else if (consigne < 0)
    {
        sens = -1;
    }
    float vitesse_croisiere_gauche = vitesse * sens;
    float vitesse_croisiere_droit = vitesse * sens;

    int consigne_gauche = consigne;
    int consigne_droite = consigne;

    consigne_gauche = (consigne_gauche + consigne_odo_gauche_prec);
    consigne_droite = (consigne_droite + consigne_odo_droite_prec);

    // Calcul initial des consignes de vitesse pour chaque roue
    consigne_regulation_vitesse_droite = regulation_vitesse_roue_folle_droite(consigne_droite, vitesse_croisiere_droit);
    consigne_regulation_vitesse_gauche = regulation_vitesse_roue_folle_gauche(consigne_gauche, vitesse_croisiere_gauche);
    // Correction d'angle basée sur l'écart entre les consignes gauche et droite
    double consigne_angle = consigne_theta_prec;     // Consigne d'angle cible (par exemple, maintenir 0°)
    double observation_angle = degrees(theta_robot); // Angle actuel du robot
    double correction = asservissement_angle_correction(consigne_angle, observation_angle);

    // // Appliquer la correction à la consigne de vitesse
    consigne_regulation_vitesse_droite -= correction;
    consigne_regulation_vitesse_gauche += correction;
}

void x_y_theta(float coordonnee_x, float coordonnee_y, float theta_fin, int vitesse)
{

    float hypothenuse = sqrt(pow(coordonnee_x, 2) + pow(coordonnee_y, 2));
    // TOA : Tangente = Opposé / Adjacent.
    theta_premiere_rotation = degrees(atan2(coordonnee_y, coordonnee_x));

    theta_deuxieme_rotation = theta_fin - theta_premiere_rotation;
    int time = 100;
    switch (etat_x_y_theta)
    {
    case -1:
        break;
    case 0:

        rotation(convert_angle_deg_to_tick(fabs(theta_premiere_rotation)), vitesse);

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

        ligne_droite((convert_distance_mm_to_tick(hypothenuse)), vitesse);

        if (return_flag_asser_roue())
        {
            stop_motors();
            delay(time);
            etat_x_y_theta = 2;
            lauch_flag_asser_roue(true);
        }
        break;
    case 2:

        rotation((convert_angle_deg_to_tick(theta_deuxieme_rotation)), vitesse);
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

float erreur_distance = 0;
float erreur_orient = 0;
float consigne_dist_polaire_tick_max = 100;
float coeff_rot_polaire_tick = 0.1;
float coeff_dist_polaire_tick = 0.5;
float consigne_rot_polaire_tick = 0;
float consigne_dist_polaire_tick = 0;

float deccelartion_distance_polaire_tick = 15.25;
float distance_decl_polaire_tick = 0;
bool calcul_distance_decl_polaire_tick = 0;

void asser_polaire_tick(float coordonnee_x, float coordonnee_y, float theta_cons)
{
    // coordonnee_x = 200;
    // coordonnee_y = 0;
    erreur_distance = convert_distance_mm_to_tick(sqrt(pow(coordonnee_x - odo_x, 2) + pow(coordonnee_y - odo_y, 2))); // On détermine la distance restante a parcourir
    erreur_orient = convert_angle_radian_to_tick((atan2(coordonnee_y - odo_y, coordonnee_x - odo_x) - theta_robot));  // On détermine l'angle a parcour pour arriver a destination


    if (erreur_distance < distance_decl_polaire_tick)
    {
        float facteur_deccel = erreur_distance / distance_decl_polaire_tick;
        consigne_dist_polaire_tick = consigne_dist_polaire_tick_max * facteur_deccel;
        if (convert_distance_tick_to_mm(erreur_distance) <= 5)
        {
            flag_fin_mvt = true;
            Serial.printf(" Vrai ");
            consigne_odo_gauche_prec = odo_tick_gauche;
            consigne_odo_droite_prec = odo_tick_droit;
            consigne_odo_x_prec = odo_x;
            consigne_odo_y_prec = odo_y;
            consigne_theta_prec = degrees(theta_robot);
        }
    }
    else
    {
        consigne_dist_polaire_tick = consigne_dist_polaire_tick_max;
    }

    consigne_rot_polaire_tick = erreur_orient;
   

    float commande_gauche = odo_tick_gauche + coeff_dist_polaire_tick * consigne_dist_polaire_tick + coeff_rot_polaire_tick * consigne_rot_polaire_tick; // commande en tick qu'on souhaite atteindre
    float commande_droite = odo_tick_droit + coeff_dist_polaire_tick * consigne_dist_polaire_tick - coeff_rot_polaire_tick * consigne_rot_polaire_tick;  // commande en tick qu'on souhaite atteindre

    asservissement_roue_folle_gauche_tick(round(commande_gauche), odo_tick_gauche); // PID en tick des roue avec pour 1ere argument la consigne et le deuxieme argument l'observation sur la roue odo
    asservissement_roue_folle_droite_tick(round(commande_droite), odo_tick_droit);  // PID en tick des roue avec pour 1ere argument la consigne et le deuxieme argument l'observation sur la roue odo

    if (calcul_distance_decl_polaire_tick == false)
    {
        calcul_distance_decl_polaire_tick = true;
        distance_decl_polaire_tick = 0.5 * pow(2, consigne_dist_polaire_tick_max / deccelartion_distance_polaire_tick) * deccelartion_distance_polaire_tick;
    }


    Serial.printf(" cs x %.1f ", coordonnee_x);
    Serial.printf(" cs_y %.1f ", coordonnee_y);

    Serial.printf(" Odo x %.1f ", odo_x);
    Serial.printf(" odo_y %.1f ", odo_y);
    Serial.printf(" teheta %.3f ", degrees(theta_robot));
    Serial.printf(" er_d %.3f ", convert_distance_tick_to_mm(erreur_distance));
    Serial.printf(" er_o %.3f ", convert_tick_to_angle_deg(erreur_orient));
    Serial.printf(" cmd_d %.1f ", consigne_dist_polaire_tick);
    Serial.printf(" cmd_r %.1f ", consigne_rot_polaire_tick);

    Serial.printf(" dist_dcl %.1f ", convert_distance_tick_to_mm(distance_decl_polaire_tick));
    Serial.printf(" coef_decl %.1f ", deccelartion_distance_polaire_tick);

    Serial.printf(" odo_gauche %.0f ", odo_tick_gauche);
    Serial.printf(" odo_droit %.0f ", odo_tick_droit);

    // Serial.printf(" angl_tick %.1f ", (float)convert_angle_deg_to_tick(90));
    // Serial.printf(" angl_deg %.1f ", convert_tick_to_angle_deg(convert_angle_deg_to_tick(90)));

    Serial.println();
}

