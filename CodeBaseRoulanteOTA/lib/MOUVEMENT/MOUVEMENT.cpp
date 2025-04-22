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

    consigne_position_droite = regulation_vitesse_roue_folle_droite(consigne_droite, vitesse_croisiere_droit);
    consigne_position_gauche = regulation_vitesse_roue_folle_gauche(consigne_gauche, vitesse_croisiere_gauche);
    if ((etat_actuel_vit_roue_folle_gauche != ETAT_DECELERATION_Vitesse_ROUE_FOLLE_GAUCHE) || (etat_actuel_vit_roue_folle_droite != ETAT_DECELERATION_Vitesse_ROUE_FOLLE_DROITE))
    {
        float ecart = consigne_odo_gauche_prec + consigne_odo_droite_prec;
        consigne_position_gauche = -consigne_position_droite + ecart;
    }
    // // // On force les consignes à être égales et opposées
    // consigne_position_droite = sens * consigne_regulation_moyenne;
    // consigne_position_gauche = -sens * consigne_regulation_moyenne;
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
    consigne_position_droite = regulation_vitesse_roue_folle_droite(consigne_droite, vitesse_croisiere_droit);
    consigne_position_gauche = regulation_vitesse_roue_folle_gauche(consigne_gauche, vitesse_croisiere_gauche);
    // Correction d'angle basée sur l'écart entre les consignes gauche et droite
    correction = asservissement_angle_correction(consigne_theta_prec, degrees(theta_robot));

    // // Appliquer la correction à la consigne de vitesse
    consigne_position_droite -= correction;
    consigne_position_gauche += correction;
}

// void x_y_theta(float coordonnee_x, float coordonnee_y, float theta_fin, int vitesse)
// {

//     float hypothenuse = sqrt(pow(coordonnee_x, 2) + pow(coordonnee_y, 2));
//     // TOA : Tangente = Opposé / Adjacent.
//     theta_premiere_rotation = degrees(atan2(coordonnee_y, coordonnee_x));

//     theta_deuxieme_rotation = theta_fin - theta_premiere_rotation;
//     int time = 100;
//     switch (etat_x_y_theta)
//     {
//     case -1:
//         break;
//     case 0:

//         rotation(convert_angle_deg_to_tick(fabs(theta_premiere_rotation)), vitesse);

//         if (return_flag_asser_roue())
//         {

//             stop_motors();
//             delay(time);
//             etat_x_y_theta = 1;
//             lauch_flag_asser_roue(true);
//             consigne_theta_prec = degrees(theta_robot);
//         }
//         break;

//     case 1:

//         ligne_droite((convert_distance_mm_to_tick(hypothenuse)), vitesse);

//         if (return_flag_asser_roue())
//         {
//             stop_motors();
//             delay(time);
//             etat_x_y_theta = 2;
//             lauch_flag_asser_roue(true);
//         }
//         break;
//     case 2:

//         rotation((convert_angle_deg_to_tick(theta_deuxieme_rotation)), vitesse);
//         if (return_flag_asser_roue())
//         {
//             stop_motors();
//             delay(time);
//             consigne_theta_prec = degrees(theta_robot);

//             etat_x_y_theta = -1;
//         }
//         break;

//     default:
//         break;
//     }
// }

void asser_polaire_tick(float coordonnee_x, float coordonnee_y, float theta_cons, bool nbr_passage)
{
    // coordonnee_x = 200;
    // coordonnee_y = 0;
    erreur_distance = convert_distance_mm_to_tick(sqrt(pow(coordonnee_x - odo_x, 2) + pow(coordonnee_y - odo_y, 2))); // On détermine la distance restante a parcourir
    // erreur_orient = convert_angle_radian_to_tick((atan2(coordonnee_y - odo_y, coordonnee_x - odo_x) - theta_robot));  // On détermine l'angle a parcour pour arriver a destination
    erreur_orient = atan2(coordonnee_y - odo_y, coordonnee_x - odo_x) - theta_robot; // On détermine l'angle a parcour pour arriver a destination
    erreur_orient = normaliser_angle_rad(erreur_orient);
    erreur_orient = convert_angle_radian_to_tick(erreur_orient);
    erreur_orient = constrain(erreur_orient, -1250, 1250);
    consigne_rot_polaire_tick = erreur_orient;

    if ((erreur_distance <= distance_decl_polaire_tick) && (nbr_passage == true))
    {
        float facteur_deccel = erreur_distance / distance_decl_polaire_tick;
        consigne_dist_polaire_tick = consigne_dist_polaire_tick_max * facteur_deccel;
        if (convert_distance_tick_to_mm(erreur_distance) <= 10.0)
        {
            Serial.printf(" Vrai ");
            consigne_odo_gauche_prec = odo_tick_gauche;
            consigne_odo_droite_prec = odo_tick_droit;
            consigne_odo_x_prec = odo_x;
            consigne_odo_y_prec = odo_y;
            consigne_theta_prec = degrees(theta_robot);
            flag_fin_mvt = true;
            calcul_decl_polaire_tick = false;
        }
        Serial.printf(" Vrai 4");
    }
    else if ((erreur_orient > convert_angle_deg_to_tick(20)) || (erreur_orient < convert_angle_deg_to_tick(-20)))
    {
        Serial.printf(" Vrai 2");
        // coeff_rot_polaire_tick = 0.5;
        consigne_dist_polaire_tick = 0;
    }
    else
    {
        Serial.printf(" Vrai 3");
        // coeff_rot_polaire_tick = 0.1;
        consigne_dist_polaire_tick = consigne_dist_polaire_tick_max;
    }

    consigne_position_gauche = odo_tick_gauche + coeff_dist_polaire_tick * consigne_dist_polaire_tick + coeff_rot_polaire_tick * consigne_rot_polaire_tick; // commande en tick qu'on souhaite atteindre
    consigne_position_droite = odo_tick_droit + coeff_dist_polaire_tick * consigne_dist_polaire_tick - coeff_rot_polaire_tick * consigne_rot_polaire_tick;  // commande en tick qu'on souhaite atteindre

    // asservissement_roue_folle_gauche_tick(round(commande_gauche), odo_tick_gauche); // PID en tick des roue avec pour 1ere argument la consigne et le deuxieme argument l'observation sur la roue odo
    // asservissement_roue_folle_droite_tick(round(commande_droite), odo_tick_droit);  // PID en tick des roue avec pour 1ere argument la consigne et le deuxieme argument l'observation sur la roue odo

    if (calcul_decl_polaire_tick == false)
    {
        calcul_decl_polaire_tick = true;
        distance_decl_polaire_tick = 0.5 * pow(2, consigne_dist_polaire_tick_max / coeff_decc_distance_polaire_tick) * coeff_decc_distance_polaire_tick;
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
    Serial.printf(" cff_r %.1f ", coeff_rot_polaire_tick);
    Serial.printf(" cff_d %.1f ", coeff_dist_polaire_tick);

    // Serial.printf(" dist_dcl %.1f ", convert_distance_tick_to_mm(distance_decl_polaire_tick));
    // Serial.printf(" coef_decl %.1f ", coeff_decc_distance_polaire_tick);

    Serial.printf(" odo_g %.0f ", odo_tick_gauche);
    Serial.printf(" odo_d %.0f ", odo_tick_droit);

    // Serial.printf(" angl_tick %.1f ", (float)convert_angle_deg_to_tick(90));
    // Serial.printf(" angl_deg %.1f ", convert_tick_to_angle_deg(convert_angle_deg_to_tick(90)));

    Serial.println();
}

bool recalage(uint8_t direction, uint8_t type_modif, uint16_t nouvelle_valeur, uint16_t consigne_rotation)
{
    bool flag_modif_fait = false;
    enum MouvementRecalage
    {
        IMMOBILE = 0,
        AVANT = 1,
        ARRIERE = 2
    };

    enum TypeModification
    {
        AUCUNE = 0,
        MODIF_X = 1,
        MODIF_Y = 2,
        MODIF_THETA = 3
    };
    int8_t sens = 0;

    // Machine à états pour le mouvement
    switch (direction)
    {
    case IMMOBILE:
        // Pas de mouvement , rien à faire ici
        break;
    case AVANT:
    case ARRIERE:

        sens = (direction == AVANT) ? 1 : -1; // condition ? valeur_si_vrai : valeur_si_faux;

        consigne_position_gauche = odo_tick_gauche + sens * SPEED_NORMAL;
        consigne_position_droite = odo_tick_droit + sens * SPEED_NORMAL;

        correction = asservissement_angle_correction(consigne_theta_prec, degrees(theta_robot));
        consigne_position_droite -= correction;
        consigne_position_gauche += correction;
        break;
    }

    // Mise à jour de l’odométrie si contact
    if (toucher_objet_solid())
    {
        if (type_modif == MODIF_X)
        {
            // odo_x = convert_distance_mm_to_tick(nouvelle_valeur);
            odo_x = nouvelle_valeur;
            flag_modif_fait = true;
            Serial.printf("modif effec x");
            // delay( 2000);
        }
        else if (type_modif == MODIF_Y)
        {
            // odo_y = convert_distance_mm_to_tick(nouvelle_valeur);
            odo_y = nouvelle_valeur;
            flag_modif_fait = true;

            Serial.printf("modif effec x");
            // delay( 2000);

        }
    }
    if (type_modif == MODIF_THETA)
    {
        theta_robot = radians(convert_angle_deg_to_tick(nouvelle_valeur));
        consigne_theta_prec = convert_angle_deg_to_tick(nouvelle_valeur);
    }

    // Rotation demandée ?
    if (consigne_rotation != 0)
    {
        Serial.printf("Déclenchement rotation\n");
        liste.general_purpose = TYPE_DEPLACEMENT_ROTATION;
        liste.angle = convert_angle_deg_to_tick(consigne_rotation);
        liste.vitesse_croisiere = SPEED_NORMAL;
        lauch_flag_asser_roue(true);
    }

    // Serial.printf(" Odo x %.3f ", odo_x);
    // Serial.printf(" odo_y %.3f ", odo_y);
    // Serial.printf(" teheta %.3f ", degrees(theta_robot));
    // Serial.printf(" consigne_position_droite %.0f ", consigne_position_droite);
    // Serial.printf(" consigne_position_gauche %.0f ", consigne_position_gauche);
    // Serial.printf(" odo_tick_droit %.0f ", odo_tick_droit);
    // Serial.printf(" odo_tick_gauche %.0f ", odo_tick_gauche);
    // Serial.println();
    return flag_modif_fait;
}
bool toucher_objet_solid()
{

    static unsigned long last_time_droite = millis();
    static unsigned long last_time_gauche = millis();
    static int last_odo_droite = odo_tick_droit;
    static int last_odo_gauche = odo_tick_gauche;
    const unsigned long timeout = 500; // Temps avant de détecter un blocage (ms)

    static bool roue_droite_bloquer = false;
    static bool roue_gauche_bloquer = false;

    if (millis() - last_time_droite > timeout)
    {
        if (last_odo_droite == odo_tick_droit)
        {
            Serial.printf("R_D block");
            roue_droite_bloquer = true;
        }

        last_odo_droite = odo_tick_droit;
        last_time_droite = millis();
    }

    if (millis() - last_time_gauche > timeout)
    {
        if (last_odo_gauche == odo_tick_gauche)
        {
            Serial.printf("R_G block");
            roue_gauche_bloquer = true;
        }

        last_odo_gauche = odo_tick_gauche;
        last_time_gauche = millis();
    }

    if ((roue_gauche_bloquer == true) && (roue_droite_bloquer == true))
    {
        return true;
    }
    else
    {
        return false;
    }
}



