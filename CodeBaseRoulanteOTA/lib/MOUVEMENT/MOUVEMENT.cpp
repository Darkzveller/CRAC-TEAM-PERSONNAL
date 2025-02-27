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

float hypothenuse_prec = 0;
float theta_parcourir_prec = 0;

float coeff_P_dist_polaire = 6.5;
float coeff_D_dist_polaire = 0.5;
float coeff_I_dist_polaire = 0.1;
float integral_limit_dist_polaire_limit = 1000;
float somme_erreur_dist_polaire = 0;

float coeff_P_orient_polaire = 2000.0; // 1100 1750 2000
float coeff_D_orient_polaire = 0;
float coeff_I_orient_polaire = 2.5;
float integral_limit_orient_polaire_limit = 1000.0;
float somme_erreur_orient_polaire = 0;
float limit_commande_dist = 1500;        // 1200
float limit_commande_orient = 1525 - 19; // 2000
float v_gauche = 0;
float v_droite = 0;
float vitesse_cible = 0;

int etat_polaire = 0;
/*
void asser_polaire(float coordonnee_x, float coordonnee_y, float theta_cons)
{
    float hypothenuse = sqrt(pow(coordonnee_x - odo_x, 2) + pow(coordonnee_y - odo_y, 2));
    float cons_hypothenuse = sqrt(pow(coordonnee_x, 2) + pow(coordonnee_y, 2));

    float theta_parcourir = atan2(coordonnee_y - odo_y, coordonnee_x - odo_x) - theta_robot;

    // 📌 Définition des paramètres pour le profil trapézoïdal
    float V_max = 200.0; // Vitesse max de croisière (modifiable)
    float A_max = 50.0;  // Accélération maximale (modifiable)
    Serial.printf(" cons_hypothenuse %.3f ", cons_hypothenuse);

    // Calcul des distances pour accélération et décélération
    float d_accel = cons_hypothenuse * 1.0 / 3.0;
    float d_decel = cons_hypothenuse * 1.0 / 3.0;
    float d_totale = cons_hypothenuse;
    Serial.printf(" d_accel %.3f ", d_accel);
    Serial.printf(" d_decel %.3f ", d_decel);
    Serial.printf(" d_totale %.3f ", d_totale);
    Serial.printf(" (cons_hypothenuse-hypothenuse) %.3f ", (cons_hypothenuse - hypothenuse));

    // // Si la distance totale est trop courte, ajuste la vitesse max
    // if (d_totale < (d_accel + d_decel))
    // {
    //     V_max = sqrt(2 * A_max * d_totale / 2);
    //     d_accel = d_totale / 2;
    //     d_decel = d_totale / 2;
    // }

    // 📌 Calcul de la vitesse cible en fonction de la phase
    switch (etat_polaire)
    {
    case 0:
        Serial.printf("Vitesse cible ACCÉLÉRATION: %.3f", vitesse_cible);
        // Accélération : v = sqrt(2 * a * d)
        vitesse_cible = sqrt(2 * A_max * d_accel);
        if ((cons_hypothenuse - hypothenuse) < d_accel)
        {
            etat_polaire = 1;
        }
        break;
    case 1:
        Serial.printf("Vitesse cible CROISIÈRE: %.3f", vitesse_cible);
        // Phase de croisière : vitesse max
        vitesse_cible = V_max;

        if ((cons_hypothenuse - hypothenuse) > (cons_hypothenuse * 2.0 / 3.0))
        {
            etat_polaire = 2;
        }
        break;
    case 2:
        Serial.printf("Vitesse cible DÉCÉLÉRATION: %.3f", vitesse_cible);
        // Décélération : v = sqrt(2 * a * (d_totale - d_parcourue))
        vitesse_cible = sqrt(2 * A_max * (cons_hypothenuse - hypothenuse));
        if(vitesse_cible<0){
            etat_polaire = 3;
        }
        break;
    case 3:
        stop_motors();
        break;

    default:
        break;
    }
    // if ((cons_hypothenuse - hypothenuse) < d_accel)
    // {
    //     Serial.printf("Vitesse cible ACCÉLÉRATION: %.3f", vitesse_cible);
    //     // Accélération : v = sqrt(2 * a * d)
    //     vitesse_cible = sqrt(2 * A_max * d_accel);
    // }
    // else if ((cons_hypothenuse - hypothenuse) > (cons_hypothenuse * 2.0 / 3.0))
    // {
    //     Serial.printf("Vitesse cible DÉCÉLÉRATION: %.3f", vitesse_cible);
    //     // Décélération : v = sqrt(2 * a * (d_totale - d_parcourue))
    //     vitesse_cible = sqrt(2 * A_max * (d_totale - hypothenuse));
    // }
    // else
    // {
    //     Serial.printf("Vitesse cible CROISIÈRE: %.3f", vitesse_cible);
    //     // Phase de croisière : vitesse max
    //     vitesse_cible = V_max;
    // }
    // 📌 PID sur la vitesse (avec la consigne de vitesse modifiée par le profil trapézoïdal)
    somme_erreur_vit_polaire += (vitesse_cible - vitesse_rob) * Te;
    if (somme_erreur_vit_polaire > integral_limit_vit_polaire_limit)
    {
        somme_erreur_vit_polaire = integral_limit_vit_polaire_limit;
    }
    else if (somme_erreur_vit_polaire < -integral_limit_vit_polaire_limit)
    {
        somme_erreur_vit_polaire = -integral_limit_vit_polaire_limit;
    }

    float commande_v_polaire = coeff_P_vit_polaire * (vitesse_cible - vitesse_rob) + (coeff_D_vit_polaire * (hypothenuse - hypothenuse_prec)) / Te + coeff_I_vit_polaire * somme_erreur_vit_polaire;

    // Limitation de la vitesse
    if (commande_v_polaire > limit_commande_vit)
    {
        commande_v_polaire = limit_commande_vit;
    }
    else if (commande_v_polaire < -limit_commande_vit)
    {
        commande_v_polaire = -limit_commande_vit;
    }

    hypothenuse_prec = hypothenuse;

    // 📌 PID pour l'orientation angulaire
    somme_erreur_w_polaire += theta_parcourir * Te;
    if (somme_erreur_w_polaire > integral_limit_w_polaire_limit)
    {
        somme_erreur_w_polaire = integral_limit_w_polaire_limit;
    }
    if (somme_erreur_w_polaire < -integral_limit_w_polaire_limit)
    {
        somme_erreur_w_polaire = -integral_limit_w_polaire_limit;
    }

    float commande_w_polaire = coeff_P_w_polaire * theta_parcourir + (coeff_D_w_polaire * (theta_parcourir - theta_parcourir_prec)) / Te + coeff_I_w_polaire * somme_erreur_w_polaire;

    if (commande_w_polaire > limit_commande_w)
    {
        commande_w_polaire = limit_commande_w;
    }
    else if (commande_w_polaire < -limit_commande_w)
    {
        commande_w_polaire = -limit_commande_w;
    }

    theta_parcourir_prec = theta_parcourir;

    // 📌 Calcul des vitesses des moteurs
    v_gauche = (commande_v_polaire + commande_w_polaire) + odo_tick_gauche;
    v_droite = (commande_v_polaire - commande_w_polaire) + odo_tick_droit;

    // 📌 Envoi des vitesses aux moteurs
    asservissement_roue_folle_droite_tick(v_droite, odo_tick_droit);
    asservissement_roue_folle_gauche_tick(v_gauche, odo_tick_gauche);

    // 📌 Affichage des données pour debug
    Serial.printf(" Odo x %.3f ", odo_x);
    Serial.printf(" odo_y %.3f ", odo_y);
    Serial.printf(" theta %.3f ", degrees(theta_robot));
    Serial.printf(" hypothenuse %.3f ", hypothenuse);
    Serial.printf(" theta_parcourir %.3f ", degrees(theta_parcourir));
    Serial.printf(" vitesse_cible %.3f ", vitesse_cible);
    // Serial.printf(" commande_v_polaire %.3f ", commande_v_polaire);
    // Serial.printf(" commande_w_polaire %.3f ", commande_w_polaire);
    // Serial.printf(" v_gauche %.3f ", v_gauche);
    // Serial.printf(" v_droite %.3f ", v_droite);
    Serial.println();
}
*/
float hypothenuse;
float cons_hypothenuse;
float theta_parcourir;

float limit_stop_dist = 72 / 1.0;
float limit_stop_orient = radians(0.1);

void asser_polaire(float coordonnee_x, float coordonnee_y, float theta_cons)
{
    hypothenuse = sqrt(pow(coordonnee_x - odo_x, 2) + pow(coordonnee_y - odo_y, 2));
    // verif_non_depassement = hypothenuse_prec - hypothenuse;
    cons_hypothenuse = sqrt(pow(coordonnee_x, 2) + pow(coordonnee_y, 2));

    // theta_parcourir = atan2(coordonnee_y - odo_y, coordonnee_x - odo_x) - theta_robot;
    // theta_parcourir =radians( theta_cons) - theta_robot;
    theta_parcourir = atan2(coordonnee_y, coordonnee_x) - theta_robot;

    somme_erreur_dist_polaire += hypothenuse * Te;

    if (somme_erreur_dist_polaire > integral_limit_dist_polaire_limit)
    {
        somme_erreur_dist_polaire = integral_limit_dist_polaire_limit;
    }
    else if (somme_erreur_dist_polaire < -integral_limit_dist_polaire_limit)
    {
        somme_erreur_dist_polaire = -integral_limit_dist_polaire_limit;
    }
    if ((hypothenuse <= limit_stop_dist))
    {
        somme_erreur_dist_polaire = 0;
    }

    float commande_dist_polaire = coeff_P_dist_polaire * hypothenuse + (coeff_D_dist_polaire * (hypothenuse - hypothenuse_prec)) / Te + coeff_I_dist_polaire * somme_erreur_dist_polaire;

    if (commande_dist_polaire > limit_commande_dist)
    {
        commande_dist_polaire = limit_commande_dist;
    }
    else if (commande_dist_polaire < -limit_commande_dist)
    {
        commande_dist_polaire = -limit_commande_dist;
    }

    hypothenuse_prec = hypothenuse;
    // on a donc la vitesse "en ligne droite" que notre robot doit avoir

    somme_erreur_orient_polaire += theta_parcourir * Te;

    if (somme_erreur_orient_polaire > integral_limit_orient_polaire_limit)
    {
        somme_erreur_orient_polaire = integral_limit_orient_polaire_limit;
    }
    else if (somme_erreur_orient_polaire < -integral_limit_orient_polaire_limit)
    {
        somme_erreur_orient_polaire = -integral_limit_orient_polaire_limit;
    }
    if ((fabs(theta_parcourir) <= limit_stop_orient))
    {
        somme_erreur_orient_polaire = 0;
        // Serial.printf("Sw 0 ");
    }

    float commande_orient_polaire = coeff_P_orient_polaire * theta_parcourir + (coeff_D_orient_polaire * (theta_parcourir - theta_parcourir_prec)) / Te + coeff_I_orient_polaire * somme_erreur_orient_polaire;

    if (commande_orient_polaire > limit_commande_orient)
    {
        commande_orient_polaire = limit_commande_orient;
    }
    else if (commande_orient_polaire < -limit_commande_orient)
    {
        commande_orient_polaire = -limit_commande_orient;
    }

    theta_parcourir_prec = theta_parcourir;

    if ((hypothenuse <= limit_stop_dist))
    {
        commande_dist_polaire = 0;

        // somme_erreur_vit_polaire = 0;
    }
    if ((fabs(theta_parcourir) <= limit_stop_orient))
    {
        commande_orient_polaire = 0;
        // somme_erreur_w_polaire = 0;
    }
    v_gauche = (commande_dist_polaire + commande_orient_polaire); // vitesse de notre moteur gauche
    v_droite = (commande_dist_polaire - commande_orient_polaire); // vitesse de notre moteur droit
    // Serial.printf(" v_droite %.3f ", v_droite);
    // Serial.printf(" v_gauche %.3f ", v_gauche);

    // asservissement_roue_folle_droite_tick(v_droite, odo_tick_droit);
    // asservissement_roue_folle_gauche_tick(v_gauche, odo_tick_gauche);

    // if ((hypothenuse <= 10))
    // {
    //     v_droite = 0;
    //     v_gauche = 0;
    //     somme_erreur_vit_polaire = 0;
    // }

    // if (theta_parcourir <= radians(2))
    // {
    //     somme_erreur_w_polaire = 0;
    // }

    moteur_gauche_polaire(-round(v_gauche));
    moteur_droit_polaire(-round(v_droite));

    // moteur_droit_polaire(-v_droite);
    // moteur_gauche_polaire(-v_gauche);
    Serial.printf(" Odo x %.1f ", odo_x);
    Serial.printf(" odo_y %.1f ", odo_y);
    Serial.printf(" teheta %.3f ", degrees(theta_robot));
    // Serial.printf(" cons_hypothenuse %.3f ", cons_hypothenuse);

    Serial.printf(" hypothenuse %.3f ", hypothenuse);
    Serial.printf(" theta_parcourir %.3f ", degrees(theta_parcourir));

    // Serial.printf(" coordonnee_x %.3f ", coordonnee_x);
    // Serial.printf(" coordonnee_y %.3f ", coordonnee_y);
    Serial.printf(" commande_dist_polaire %.3f ", commande_dist_polaire);
    Serial.printf(" commande_orient_polaire %.3f ", commande_orient_polaire);
    Serial.printf(" v_g %.1f ", v_gauche);
    Serial.printf(" v_d %.1f ", v_droite);

    // Serial.printf(" odo_tick_gauche %.3f ", odo_tick_gauche);
    // Serial.printf(" odo_tick_droit %.3f ", odo_tick_droit);
    Serial.printf(" somme_err_dist %.3f ", somme_erreur_dist_polaire);
    Serial.printf(" C+somme_err_orient %.3f ", coeff_I_orient_polaire * somme_erreur_orient_polaire);

    // Serial.printf(" consigne_regulation_vitesse_gauche %.3f ", consigne_regulation_vitesse_gauche);
    // Serial.printf(" consigne_regulation_vitesse_droite %.3f ", consigne_regulation_vitesse_droite);

    Serial.println();
}

// float erreur_dist;
// float somme_erreur_dist;
// float delta_erreur_dist;
// float erreur_prec_dist;
// float erreur_orient;
// float somme_erreur_orient;
// float delta_erreur_orient;
// float erreur_prec_orient;
// float kp_dist;
// float ki_dist;
// float kd_dist;
// float kp_orient;
// float ki_orient;
// float kd_orient;
// void pid(float ConsigneDist, float ConsigneOrient)
// {
//     float compteurD;
//     float compteurG;
//     float mesure_dist;
//     float mesure_orient;
//     float cmd_dist;
//     float cmd_orient;

//     // affectation des compteurs de PID et reset des compteurs sur interruption
//     // compteurD = tickD;
//     // compteurG = tickG;
//     // tickD = 0;
//     // tickG = 0;

//     // mesure distance et orientation
//     mesure_dist = (odo_tick_droit + odo_tick_gauche) / 2;
//     mesure_orient = odo_tick_droit - odo_tick_gauche;

//     // Calcul des erreurs de distance
//     erreur_dist = ConsigneDist - mesure_dist;
//     somme_erreur_dist += erreur_dist;
//     delta_erreur_dist = erreur_dist - erreur_prec_dist;
//     // mise à jour de l'erreur précédente
//     erreur_prec_dist = erreur_dist;

//     // Calcul des erreurs d'orientation
//     erreur_orient = ConsigneOrient - mesure_orient;
//     somme_erreur_orient += erreur_orient;
//     delta_erreur_orient = erreur_orient - erreur_prec_orient;
//     // mise à jour de l'erreur précédente
//     erreur_prec_orient = erreur_orient;

//     // calcul des commandes
//     cmd_dist = ((kp_dist * erreur_dist) + (ki_dist * somme_erreur_dist) + (kd_dist * delta_erreur_dist));               // PID distance
//     cmd_orient = ((kp_orient * erreur_orient) + (ki_orient * somme_erreur_orient) + (kd_orient * delta_erreur_orient)); // PID orientation

//     // appliquer les commandes aux moteur
//     float PWMG = cmd_dist - cmd_orient; // ça ne doit surement pas fonctionner de cette façon
//     float PWMD = cmd_dist + cmd_orient;

//     // Normalisation des commandes PWM de sortie (le moteur ne bouge pas avec un pwm < 240)
//     if (PWMD < 600)
//     {
//         PWMD = 600;
//     }
//     else if (PWMD > 3075)
//     {
//         PWMD = 3075;
//     }
//     if (PWMG < 600)
//     {
//         PWMG = 600;
//     }
//     else if (PWMG > 3075)
//     {
//         PWMG = 3075;
//     }
// }
