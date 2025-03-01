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

float commande_dist_polaire = 0;
float coeff_P_dist_polaire = 0.95; // 6.5
float coeff_D_dist_polaire = 0;    // 0.5
float coeff_I_dist_polaire = 0;    // 0.1
float integral_limit_dist_polaire_limit = 1000;
float somme_erreur_dist_polaire = 0;

float commande_orient_polaire = 0;
float coeff_P_orient_polaire = 650; // 1100 1750 2000 1250
float coeff_D_orient_polaire = 0;
float coeff_I_orient_polaire = 0; // 2.5
float integral_limit_orient_polaire_limit = 1000.0;
float somme_erreur_orient_polaire = 0;
float limit_commande_dist = 1500;                  // 1200
float limit_commande_orient = limit_commande_dist; // 1525 - 19
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

// float limit_stop_dist = 72 / 1.0;
float limit_stop_dist = 25;

float limit_stop_orient = radians(10);

enum ETAT_PROFIL_TRAPEZE_POLAIRE
{
    ETAT_ACCELERATION_PROFIL_TRAPEZE_POLAIRE,
    ETAT_CROISIERE_PROFIL_TRAPEZE_POLAIRE,
    ETAT_DECELERATION_PROFIL_TRAPEZE_POLAIRE,
    ETAT_ARRET_PROFIL_TRAPEZE_POLAIRE
};

ETAT_PROFIL_TRAPEZE_POLAIRE etat_profil_trapeze_polaire;
String toStringPolaire(ETAT_PROFIL_TRAPEZE_POLAIRE etat)
{
    switch (etat)
    {
    case ETAT_ACCELERATION_PROFIL_TRAPEZE_POLAIRE:
        return "ETAT_ACCELERATION_PROFIL_TRAPEZE_POLAIRE";
    case ETAT_CROISIERE_PROFIL_TRAPEZE_POLAIRE:
        return "ETAT_CROISIERE_PROFIL_TRAPEZE_POLAIRE";
    case ETAT_DECELERATION_PROFIL_TRAPEZE_POLAIRE:
        return "ETAT_DECELERATION_PROFIL_TRAPEZE_POLAIRE";
    case ETAT_ARRET_PROFIL_TRAPEZE_POLAIRE:
        return "ETAT_ARRET_PROFIL_TRAPEZE_POLAIRE";
    default:
        return "ETAT_INCONNU";
    }
}

float Vmax_polaire = limit_commande_dist; // 6.5
float Amax_polaire = 0.65;                 // 0.65
float Dmax_polaire =0.25;
float acc_actuel_polaire = 0;
float Ta_counter_polaire = 0;

float distance_accel_polaire = 0;
float distance_decl_polaire = 0;

float tension_reference_test = 12.5;
void asser_polaire(float coordonnee_x, float coordonnee_y, float theta_cons)
{
    static int i = 0;
    if (i == 0)
    {
        etat_profil_trapeze_polaire = ETAT_ACCELERATION_PROFIL_TRAPEZE_POLAIRE;
        i = 1;
    }

    coeff_P_dist_polaire = 8; // 5.01
    coeff_D_dist_polaire = 0;
    coeff_I_dist_polaire = 0;

    coeff_P_orient_polaire = 2400;
    coeff_D_orient_polaire = 0;
    coeff_I_orient_polaire = 0;

    hypothenuse = sqrt(pow(coordonnee_x - odo_x, 2) + pow(coordonnee_y - odo_y, 2));
    cons_hypothenuse = sqrt(pow(coordonnee_x, 2) + pow(coordonnee_y, 2));
    theta_parcourir = atan2(coordonnee_y - odo_y, coordonnee_x - odo_x) - theta_robot;
    // theta_parcourir =radians( theta_cons) - theta_robot;
    // theta_parcourir = atan2(coordonnee_y, coordonnee_x) - theta_robot;
    float vit = Vmax_polaire;
    float accel = Amax_polaire;
    float decc = Dmax_polaire;

    float distance_restante;
    float Ta = vit / accel;
    float Td = vit / decc;
    float Tc = (2.0 * cons_hypothenuse - accel * (Ta * Ta + Td * Td)) / (2 * vit);
    distance_accel_polaire = 0.5 * Ta * Ta * accel;
    distance_decl_polaire = 0.5 * Td * Td * decc;

    somme_erreur_dist_polaire += hypothenuse * Te;

    somme_erreur_dist_polaire += hypothenuse * Te;
    somme_erreur_dist_polaire = constrain(somme_erreur_dist_polaire, -integral_limit_dist_polaire_limit, integral_limit_dist_polaire_limit);

    commande_dist_polaire = coeff_P_dist_polaire * hypothenuse + (coeff_D_dist_polaire * (hypothenuse - hypothenuse_prec)) / Te + coeff_I_dist_polaire * somme_erreur_dist_polaire;
    commande_dist_polaire = constrain(commande_dist_polaire, -limit_commande_dist, limit_commande_dist);

    hypothenuse_prec = hypothenuse;
    // on a donc la vitesse "en ligne droite" que notre robot doit avoir

    somme_erreur_orient_polaire += theta_parcourir * Te;

    somme_erreur_orient_polaire = constrain(somme_erreur_orient_polaire, -integral_limit_orient_polaire_limit, integral_limit_orient_polaire_limit);

    commande_orient_polaire = coeff_P_orient_polaire * theta_parcourir + (coeff_D_orient_polaire * (theta_parcourir - theta_parcourir_prec)) / Te + coeff_I_orient_polaire * somme_erreur_orient_polaire;

    commande_orient_polaire = constrain(commande_orient_polaire, -limit_commande_orient, limit_commande_orient);

    theta_parcourir_prec = theta_parcourir;
    // Globalement le switch case suivant est censé effectuer un profil trapézoidal mais en fin de compte il ne serta pas a grand chose
    switch (etat_profil_trapeze_polaire)
    {

    case ETAT_ACCELERATION_PROFIL_TRAPEZE_POLAIRE:
        Serial.printf(" ETAT_ACCELERATION_PROFIL_TRAPEZE_POLAIRE ");
        acc_actuel_polaire = acc_actuel_polaire + vitesse_rob * Te;
        commande_dist_polaire = vitesse_rob + acc_actuel_polaire * Te;

        acc_actuel_polaire = constrain(acc_actuel_polaire, -accel, accel);
        commande_dist_polaire = constrain(commande_dist_polaire, -Vmax_polaire, Vmax_polaire);

        Ta_counter_polaire++;
        if (Ta_counter_polaire > Ta)
        {
            etat_profil_trapeze_polaire = ETAT_CROISIERE_PROFIL_TRAPEZE_POLAIRE;
        }
        break;

    case ETAT_CROISIERE_PROFIL_TRAPEZE_POLAIRE:
        // commande_dist_polaire = 1200; // Je sais pas trop quoi faire avec lui j'ai l'impression que ca fonctionne mieux avec lui
        if (hypothenuse < fabs(distance_decl_polaire))
        {
            etat_profil_trapeze_polaire = ETAT_DECELERATION_PROFIL_TRAPEZE_POLAIRE;
        }
        Serial.printf(" ETAT_CROISIERE_PROFIL_TRAPEZE_POLAIRE ");

        break;

    case ETAT_DECELERATION_PROFIL_TRAPEZE_POLAIRE:

        if (commande_dist_polaire < 600) // Inférieur a 600, car en dessous de cette valeur le moteur n'a pas la force de faire avancer le robot
        {
            etat_profil_trapeze_polaire = ETAT_ARRET_PROFIL_TRAPEZE_POLAIRE;
        }
        else
        {
            commande_dist_polaire = commande_dist_polaire - decc * Te;
        }
        Serial.printf(" ETAT_DECELERATION_PROFIL_TRAPEZE_POLAIRE ");

        break;

    case ETAT_ARRET_PROFIL_TRAPEZE_POLAIRE:
        commande_dist_polaire = 0;
        Serial.printf(" ETAT_ARRET_PROFIL_TRAPEZE_POLAIRE ");

        break;
    }

    v_gauche = (commande_dist_polaire + commande_orient_polaire); // vitesse de notre moteur gauche
    v_droite = (commande_dist_polaire - commande_orient_polaire); // vitesse de notre moteur droit

    moteur_gauche_polaire(-round(v_gauche));
    moteur_droit_polaire(-round(v_droite));

    // Pour effectuer une sauvegarde pour les autres fonctions car oui ca fonctionne et non c'est pas le centre du monde
    consigne_odo_gauche_prec = odo_tick_gauche;
    consigne_odo_droite_prec = odo_tick_droit;
    consigne_odo_x_prec = odo_x;
    consigne_odo_y_prec = odo_y;
    consigne_theta_prec = degrees(theta_robot);
    consigne_regulation_vitesse_droite = odo_tick_droit;
    consigne_regulation_vitesse_gauche = odo_tick_gauche;

    if (hypothenuse < 20)
    {
        flag_fin_mvt = true;
        Serial.printf(" Vrai ");
    }
    Serial.printf(" Odo x %.1f ", odo_x);
    Serial.printf(" odo_y %.1f ", odo_y);
    Serial.printf(" teheta %.3f ", degrees(theta_robot));
    Serial.printf(" hypothenuse %.3f ", hypothenuse);
    Serial.printf(" cons_hypothenuse %.3f ", cons_hypothenuse);
    Serial.printf(" theta_parcourir %.3f ", degrees(theta_parcourir));
    Serial.printf(" cmd_d_p %.3f ", commande_dist_polaire);
    Serial.printf(" cmd_w_p %.3f ", commande_orient_polaire);

    // // Serial.printf(" distance_accel_polaire %.3f ", distance_accel_polaire);
    Serial.printf(" distance_decl_polaire %.3f ", distance_decl_polaire);
    // Serial.printf(" Ta %.3f ", Ta);
    // Serial.printf(" Td %.3f ", Td);
    // Serial.printf(" Tc %.3f ", Tc);
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
