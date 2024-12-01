#include "Variable.h" // Remonte d'un niveau pour atteindre lib
#include "MOTEUR.h"
#include "ASSERVISSEMENT.h"
#include "OTA.h"
float facteur_ajustement_consigne = 0.5;
extern bool stop_asservissement_roue_gauche;
extern bool stop_asservissement_roue_droite;
extern bool start_asservissement_roue_gauche;
extern bool start_asservissement_roue_droite;
extern float consigne_odo_droite_delta;
extern float consigne_odo_gauche_delta;

void asservissement_roue_folle_droite_tick(double consigne, double observation)
{
    bool sens = 0;
    double sortie_roue_folle;

    double resolution_calculer = (pow(2, resolution_pwm) - 1) * POURCENT_MAX_PWM;
    double erreur = consigne - observation;
    double proportionnel = erreur * coeff_P_roue_folle_tick_droite;

    double deriver = coeff_D_roue_folle_tick_droite * (erreur - erreur_prec_roue_folle_droite_tick) / Te;

    somme_integral_roue_folle_droite_tick += erreur * Te;
    if (somme_integral_roue_folle_droite_tick > integral_limit_roue_folle_tick)
    {
        somme_integral_roue_folle_droite_tick = integral_limit_roue_folle_tick;
    }
    else if (somme_integral_roue_folle_droite_tick < -integral_limit_roue_folle_tick)
    {
        somme_integral_roue_folle_droite_tick = -integral_limit_roue_folle_tick;
    }

    double integral = coeff_I_roue_folle_tick_droite * somme_integral_roue_folle_droite_tick;

    double commande = proportionnel + deriver + integral;

    erreur_prec_roue_folle_droite_tick = erreur;

    // Gestion des bornes de la commande
    if (commande > 0)
    {
        sens = false;
        if (commande > resolution_calculer)
        {
            sortie_roue_folle = resolution_calculer;
        }
        else
        {
            sortie_roue_folle = commande;
        }
    }
    else
    {
        sens = true;
        if (commande < -resolution_calculer)
        {

            sortie_roue_folle = resolution_calculer;
        }
        else
        {
            sortie_roue_folle = -commande;
        }
    }
    moteur_droit(sortie_roue_folle, sens);
}

void asservissement_roue_folle_gauche_tick(double consigne, double observation)
{
    bool sens = 0;
    double sortie_roue_folle;

    double resolution_calculer = (pow(2, resolution_pwm) - 1) * POURCENT_MAX_PWM;
    double erreur = consigne - observation;

    double proportionnel = erreur * coeff_P_roue_folle_tick_gauche;

    double deriver = coeff_D_roue_folle_tick_gauche * (erreur - erreur_prec_roue_folle_gauche_tick) / Te;

    somme_integral_roue_folle_gauche_tick += erreur * Te;
    if (somme_integral_roue_folle_gauche_tick > integral_limit_roue_folle_tick)
    {
        somme_integral_roue_folle_gauche_tick = integral_limit_roue_folle_tick;
    }
    else if (somme_integral_roue_folle_gauche_tick < -integral_limit_roue_folle_tick)
    {
        somme_integral_roue_folle_gauche_tick = -integral_limit_roue_folle_tick;
    }
    // if ((erreur < 1) || (erreur > -1))
    // {

    //   somme_integral_roue_folle_gauche_tick = 0;
    // }

    double integral = coeff_I_roue_folle_tick_gauche * somme_integral_roue_folle_gauche_tick;

    double commande = proportionnel + deriver + integral;

    erreur_prec_roue_folle_gauche_tick = erreur;

    // Gestion des bornes de la commande
    if (commande > 0)
    {
        sens = false;
        if (commande > resolution_calculer)
        {
            sortie_roue_folle = resolution_calculer;
        }
        else
        {
            sortie_roue_folle = commande;
        }
    }
    else
    {
        sens = true;
        if (commande < -resolution_calculer)
        {

            sortie_roue_folle = resolution_calculer;
        }
        else
        {
            sortie_roue_folle = -commande;
        }
    }

    moteur_gauche(sortie_roue_folle, sens);
}

double regulation_vitesse_roue_folle_droite(float cons, float Vmax_consigne)
{
    if (type_ligne_droite)
    {
        cons = cons * facteur_ajustement_consigne;
    }
    float erreur_vit;
    float vit = Vmax_consigne;
    float accel = Amax;
    float decc = Dmax;

    double Vrob = (delta_droit) / Te;

    float Ta = vit / accel;
    float Td = vit / decc;
    float Tc = (2.0 * cons - accel * (Ta * Ta + Td * Td)) / (2 * vit);
    distance_accel_droite = 0.5 * Ta * Ta * accel;
    distance_decl_droite = 0.5 * Td * Td * decc;

    erreur_vit = vit - (Vrob * Vmax);
    somme_erreur_vit_roue_folle_droite += erreur_vit * Te;

    if (somme_erreur_vit_roue_folle_droite > integral_limit)
    {
        somme_erreur_vit_roue_folle_droite = integral_limit;
    }
    else if (somme_erreur_vit_roue_folle_droite < -integral_limit)
    {
        somme_erreur_vit_roue_folle_droite = -integral_limit;
    }

    float derivee_erreur_vit = (erreur_vit - erreur_vit_precedente_roue_folle_droite) / Te;
    erreur_vit_precedente_roue_folle_droite = erreur_vit;
    float commande_vit = kp_vit * erreur_vit + ki_vit * somme_erreur_vit_roue_folle_droite + kd_vit * derivee_erreur_vit;
    switch (etat_actuel_vit_roue_folle_droite)
    {
    case ETAT_ATTENTE_Vitesse_ROUE_FOLLE_DROITE:
        if (start_asservissement_roue_droite)
        {
            etat_actuel_vit_roue_folle_droite = ETAT_ACCELERATION_Vitesse_ROUE_FOLLE_DROITE;
        }
        break;
    case ETAT_ACCELERATION_Vitesse_ROUE_FOLLE_DROITE:
        acc_actuel_droite = acc_actuel_droite + commande_vit * Te;

        if (acc_actuel_droite > accel)
        {
            acc_actuel_droite = accel;
        }
        else if (acc_actuel_droite < -accel)
        {
            acc_actuel_droite = -accel;
        }

        consigne_vit_droite = Vrob + acc_actuel_droite * Te;

        if (consigne_vit_droite > Vmax_consigne)
        {
            consigne_vit_droite = Vmax_consigne;
        }
        else if (consigne_vit_droite < Vmax_consigne)
        {
            consigne_vit_droite = Vmax_consigne;
        }

        consigne_dist_droite = odo_tick_droit + consigne_vit_droite * Te;

        Ta_counter_droite++;
        if (Ta_counter_droite >= fabs(Ta))
        {
            etat_actuel_vit_roue_folle_droite = ETAT_CROISIERE_Vitesse_ROUE_FOLLE_DROITE;
        }
        break;

    case ETAT_CROISIERE_Vitesse_ROUE_FOLLE_DROITE:
        consigne_vit_droite = vit;

        if (consigne_vit_droite > Vmax_consigne)
        {
            consigne_vit_droite = Vmax_consigne;
        }
        else if (consigne_vit_droite < Vmax_consigne)
        {
            consigne_vit_droite = Vmax_consigne;
        }

        consigne_dist_droite = odo_tick_droit + consigne_vit_droite * Te;

        if (fabs(cons - odo_tick_droit) < fabs(distance_decl_droite))
        {
            etat_actuel_vit_roue_folle_droite = ETAT_DECELERATION_Vitesse_ROUE_FOLLE_DROITE;
        }
        break;

    case ETAT_DECELERATION_Vitesse_ROUE_FOLLE_DROITE:
        /* acc_actuel_droite = acc_actuel_droite - commande_vit * Te;
         if (decc > 0)
         {
             if (acc_actuel_droite < 0)
             {
                 acc_actuel_droite = 0;
             }
         }
         else if (decc < 0)
         {

             if (acc_actuel_droite > 0)
             {
                 acc_actuel_droite = 0;
             }
         }

         consigne_vit_droite = Vrob - acc_actuel_droite * Te;

         if (consigne_vit_droite > Vmax_consigne)
         {
             consigne_vit_droite = Vmax_consigne;
         }
         else if (consigne_vit_droite < -Vmax_consigne)
         {
             consigne_vit_droite = Vmax_consigne;
         }

         consigne_dist_droite = odo_tick_droit + consigne_vit_droite * Te;

 */
        // stop_motors();
        // freinage_moteur_droit(true, Vmax_consigne);

        // if (((cons - odo_tick_droit) < limit_reprise_asser) || ((cons - odo_tick_droit) > -limit_reprise_asser))
        // {
        //     etat_actuel_vit_roue_folle_droite = ETAT_ARRET_Vitesse_ROUE_FOLLE_DROITE;
        // }
        // Serial.printf(" ETAT_DECELERATION_Vitesse_ROUE_FOLLE_DROITE ");
        /*
                // Utiliser le freinage actif si la décélération naturelle est insuffisante
                if (consigne_vit_droite > 0)
                {
                    // Réduire la vitesse rapidement avec le freinage moteur
                    freinage_moteur_droit(true, consigne_vit_droite);

                    // Réduire progressivement la consigne de vitesse
                    consigne_vit_droite -= fabs(decc * Te);

                    if (consigne_vit_droite < 0)
                    {
                        consigne_vit_droite = 0; // Empêcher des vitesses négatives
                    }
                }
        */
        // Si la vitesse est positive, on applique un freinage actif classique
        if (consigne_vit_droite > 0)
        {
            // Réduire la vitesse rapidement avec le freinage moteur
            freinage_moteur_droit(true, consigne_vit_droite);

            // Réduire progressivement la consigne de vitesse
            consigne_vit_droite -= fabs(decc * Te);

            // Empêcher des vitesses négatives
            if (consigne_vit_droite < 0)
            {
                consigne_vit_droite = 0;
            }
        }
        // Si la vitesse est négative (freinage inverse), appliquer le freinage inverse
        else if (consigne_vit_droite < 0)
        {
            // Freinage inverse pour ralentir la vitesse négative
            freinage_moteur_droit(false, fabs(consigne_vit_droite));

            // Augmenter la vitesse négative progressivement (décélération inverse)
            consigne_vit_droite += fabs(decc * Te);

            // S'assurer que la vitesse ne dépasse pas la valeur maximale en sens inverse
            if (consigne_vit_droite > 0)
            {
                consigne_vit_droite = 0; // Arrêter complètement si la vitesse atteint 0
            }
        }

        // Mise à jour de la consigne de distance
        consigne_dist_droite = odo_tick_droit + consigne_vit_droite * Te;

        // Vérifier si on est assez proche de la destination pour arrêter
        if (((cons - odo_tick_droit) < limit_reprise_asser) || ((cons - odo_tick_droit) > -limit_reprise_asser))
        {
            stop_moteur_droit();
            etat_actuel_vit_roue_folle_droite = ETAT_ARRET_Vitesse_ROUE_FOLLE_DROITE;
        }

        break;

    case ETAT_ARRET_Vitesse_ROUE_FOLLE_DROITE:
        if (type_ligne_droite)
        {
            consigne_dist_droite = cons * 1 / facteur_ajustement_consigne;
        }
        T_counter_attente_droite++;
        // if (T_counter_attente_droite > T_attente_droite)
        consigne_odo_droite_delta = delta_droit;
        if (consigne_odo_droite_delta == 0)
        {

            consigne_dist_droite = cons;
            start_asservissement_roue_droite = 0;
            consigne_odo_droite_prec = odo_tick_droit;

            // coeff_P_roue_folle_tick_droite = 7.0 / 2;
            // coeff_D_roue_folle_tick_droite = 0.25 / 2;
            // coeff_I_roue_folle_tick_droite = 0.3 * 2;
            etat_actuel_vit_roue_folle_droite = ETAT_VIDE_Vitesse_ROUE_FOLLE_DROITE;
        }
        else
        {
            stop_moteur_droit();
            consigne_dist_droite = odo_tick_droit + 0 * Te;
            // coeff_P_roue_folle_tick_droite = 7.0 / 50;
            // coeff_D_roue_folle_tick_droite = 0;
            // coeff_I_roue_folle_tick_droite = 0.3 * 2;
        }
        // }
        // else
        // {
        //     etat_actuel_vit_roue_folle_droite = ETAT_VIDE_Vitesse_ROUE_FOLLE_DROITE;
        // }

        break;

    case ETAT_VIDE_Vitesse_ROUE_FOLLE_DROITE:

        etat_actuel_vit_roue_folle_droite = ETAT_ATTENTE_Vitesse_ROUE_FOLLE_DROITE;

        break;
    }

    return consigne_dist_droite;
}

double regulation_vitesse_roue_folle_gauche(float cons, float Vmax_consigne)
{
    if (type_ligne_droite)
    {
        cons = cons * facteur_ajustement_consigne;
    }
    float erreur_vit;
    float vit = Vmax_consigne;
    float accel = Amax;
    float decc = Dmax;

    double Vrob = (delta_gauche) / Te;

    float Ta = vit / accel;
    float Td = vit / decc;
    float Tc = (2.0 * cons - accel * (Ta * Ta + Td * Td)) / (2 * vit);
    distance_accel_gauche = 0.5 * Ta * Ta * accel;
    distance_decl_gauche = 0.5 * Td * Td * decc;

    erreur_vit = vit - (Vrob * Vmax);
    somme_erreur_vit_roue_folle_gauche += erreur_vit * Te;

    if (somme_erreur_vit_roue_folle_gauche > integral_limit)
    {
        somme_erreur_vit_roue_folle_gauche = integral_limit;
    }
    else if (somme_erreur_vit_roue_folle_gauche < -integral_limit)
    {
        somme_erreur_vit_roue_folle_gauche = -integral_limit;
    }

    float derivee_erreur_vit = (erreur_vit - erreur_vit_precedente_roue_folle_gauche) / Te;
    erreur_vit_precedente_roue_folle_gauche = erreur_vit;
    float commande_vit = kp_vit * erreur_vit + ki_vit * somme_erreur_vit_roue_folle_gauche + kd_vit * derivee_erreur_vit;
    switch (etat_actuel_vit_roue_folle_gauche)
    {
    case ETAT_ATTENTE_Vitesse_ROUE_FOLLE_GAUCHE:
        if (start_asservissement_roue_gauche)
        {
            etat_actuel_vit_roue_folle_gauche = ETAT_ACCELERATION_Vitesse_ROUE_FOLLE_GAUCHE;
        }
        break;
    case ETAT_ACCELERATION_Vitesse_ROUE_FOLLE_GAUCHE:
        acc_actuel_gauche = acc_actuel_gauche + commande_vit * Te;

        if (acc_actuel_gauche > accel)
        {
            acc_actuel_gauche = accel;
        }
        else if (acc_actuel_gauche < -accel)
        {
            acc_actuel_gauche = -accel;
        }

        consigne_vit_gauche = Vrob + acc_actuel_gauche * Te;

        if (consigne_vit_gauche > Vmax_consigne)
        {
            consigne_vit_gauche = Vmax_consigne;
        }
        else if (consigne_vit_gauche < Vmax_consigne)
        {
            consigne_vit_gauche = Vmax_consigne;
        }

        consigne_dist_gauche = odo_tick_gauche + consigne_vit_gauche * Te;

        Ta_counter_gauche++;
        if (Ta_counter_gauche >= fabs(Ta))
        {
            etat_actuel_vit_roue_folle_gauche = ETAT_CROISIERE_Vitesse_ROUE_FOLLE_GAUCHE;
        }
        break;

    case ETAT_CROISIERE_Vitesse_ROUE_FOLLE_GAUCHE:
        consigne_vit_gauche = vit;

        if (consigne_vit_gauche > Vmax_consigne)
        {
            consigne_vit_gauche = Vmax_consigne;
        }
        else if (consigne_vit_gauche < Vmax_consigne)
        {
            consigne_vit_gauche = Vmax_consigne;
        }

        consigne_dist_gauche = odo_tick_gauche + consigne_vit_gauche * Te;

        if (fabs(cons - odo_tick_gauche) < fabs(distance_decl_gauche))
        {
            etat_actuel_vit_roue_folle_gauche = ETAT_DECELERATION_Vitesse_ROUE_FOLLE_GAUCHE;
        }
        break;

    case ETAT_DECELERATION_Vitesse_ROUE_FOLLE_GAUCHE:
        if (consigne_vit_gauche > 0)
        {
            freinage_moteur_gauche(true, consigne_vit_gauche);

            consigne_vit_gauche -= fabs(decc * Te);

            if (consigne_vit_gauche < 0)
            {
                consigne_vit_gauche = 0;
            }
        }
        else if (consigne_vit_gauche < 0)
        {
            freinage_moteur_gauche(false, fabs(consigne_vit_gauche));

            consigne_vit_gauche += fabs(decc * Te);

            if (consigne_vit_gauche > 0)
            {
                consigne_vit_gauche = 0;
            }
        }

        consigne_dist_gauche = odo_tick_gauche + consigne_vit_gauche * Te;

        if (((cons - odo_tick_gauche) < limit_reprise_asser) || ((cons - odo_tick_gauche) > -limit_reprise_asser))
        {
            stop_moteur_gauche();
            etat_actuel_vit_roue_folle_gauche = ETAT_ARRET_Vitesse_ROUE_FOLLE_GAUCHE;
        }

        break;

    case ETAT_ARRET_Vitesse_ROUE_FOLLE_GAUCHE:
        if (type_ligne_droite)
        {
            consigne_dist_gauche = cons * 1 / facteur_ajustement_consigne;
        }
        T_counter_attente_gauche++;
        consigne_odo_gauche_delta = delta_gauche;
        if (consigne_odo_gauche_delta == 0)
        {

            consigne_dist_gauche = cons;
            start_asservissement_roue_gauche = 0;
            consigne_odo_gauche_prec = odo_tick_gauche;

            etat_actuel_vit_roue_folle_gauche = ETAT_VIDE_Vitesse_ROUE_FOLLE_GAUCHE;
        }
        else
        {
            stop_moteur_gauche();
            consigne_dist_gauche = odo_tick_gauche + 0 * Te;
        }

        break;

    case ETAT_VIDE_Vitesse_ROUE_FOLLE_GAUCHE:

        etat_actuel_vit_roue_folle_gauche = ETAT_ATTENTE_Vitesse_ROUE_FOLLE_GAUCHE;

        break;
    }

    return consigne_dist_gauche;
}

        // double regulation_vitesse_roue_folle_gauche(float cons, float Vmax_consigne)
        // {
        //     if (type_ligne_droite)
        //     {
        //         cons = cons * facteur_ajustement_consigne;
        //     }

        //     float erreur_vit;
        //     float vit = Vmax_consigne;
        //     float accel = Amax;
        //     float decc = Dmax;

        //     double Vrob = (delta_gauche) / Te;

        //     float Ta = vit / accel;
        //     float Td = vit / decc;
        //     float Tc = (2.0 * cons - accel * (Ta * Ta + Td * Td)) / (2 * vit);
        //     distance_accel_gauche = 0.5 * Ta * Ta * accel;
        //     distance_decl_gauche = 0.5 * Td * Td * decc;

        //     erreur_vit = vit - (Vrob * Vmax);
        //     somme_erreur_vit_roue_folle_gauche += erreur_vit * Te;

        //     if (somme_erreur_vit_roue_folle_gauche > integral_limit)
        //     {
        //         somme_erreur_vit_roue_folle_gauche = integral_limit;
        //     }
        //     else if (somme_erreur_vit_roue_folle_gauche < -integral_limit)
        //     {
        //         somme_erreur_vit_roue_folle_gauche = -integral_limit;
        //     }

        //     float derivee_erreur_vit = (erreur_vit - erreur_vit_precedente_roue_folle_gauche) / Te;
        //     erreur_vit_precedente_roue_folle_gauche = erreur_vit;
        //     float commande_vit = kp_vit * erreur_vit + ki_vit * somme_erreur_vit_roue_folle_gauche + kd_vit * derivee_erreur_vit;
        //     switch (etat_actuel_vit_roue_folle_gauche)
        //     {
        //     case ETAT_ATTENTE_Vitesse_ROUE_FOLLE_DROITE:
        //         if (start_asservissement_roue_gauche)
        //         {
        //             etat_actuel_vit_roue_folle_gauche = ETAT_ACCELERATION_Vitesse_ROUE_FOLLE_GAUCHE;
        //         }
        //         break;
        //     case ETAT_ACCELERATION_Vitesse_ROUE_FOLLE_GAUCHE:
        //         acc_actuel_gauche = acc_actuel_gauche + commande_vit * Te;

        //         if (acc_actuel_gauche > accel)
        //         {
        //             acc_actuel_gauche = accel;
        //         }
        //         else if (acc_actuel_gauche < -accel)
        //         {
        //             acc_actuel_gauche = -accel;
        //         }

        //         consigne_vit_gauche = Vrob + acc_actuel_gauche * Te;

        //         if (consigne_vit_gauche > Vmax_consigne)
        //         {
        //             consigne_vit_gauche = Vmax_consigne;
        //         }
        //         else if (consigne_vit_gauche < Vmax_consigne)
        //         {
        //             consigne_vit_gauche = Vmax_consigne;
        //         }

        //         consigne_dist_gauche = odo_tick_gauche + consigne_vit_gauche * Te;

        //         Ta_counter_gauche++;
        //         if (Ta_counter_gauche >= fabs(Ta))
        //         {
        //             etat_actuel_vit_roue_folle_gauche = ETAT_CROISIERE_Vitesse_ROUE_FOLLE_GAUCHE;
        //         }
        //         break;

        //     case ETAT_CROISIERE_Vitesse_ROUE_FOLLE_GAUCHE:
        //         consigne_vit_gauche = vit;

        //         if (consigne_vit_gauche > Vmax_consigne)
        //         {
        //             consigne_vit_gauche = Vmax_consigne;
        //         }
        //         else if (consigne_vit_gauche < Vmax_consigne)
        //         {
        //             consigne_vit_gauche = Vmax_consigne;
        //         }

        //         consigne_dist_gauche = odo_tick_gauche + consigne_vit_gauche * Te;
        //         if (fabs(cons - odo_tick_gauche) < fabs(distance_decl_gauche))
        //         {
        //             etat_actuel_vit_roue_folle_gauche = ETAT_DECELERATION_Vitesse_ROUE_FOLLE_GAUCHE;
        //         }

        //         break;

        //     case ETAT_DECELERATION_Vitesse_ROUE_FOLLE_GAUCHE:
        //         /* acc_actuel_gauche = acc_actuel_gauche - commande_vit * Te;
        //          if (decc > 0)
        //          {
        //              if (acc_actuel_gauche < 0)
        //              {
        //                  acc_actuel_gauche = 0;
        //              }
        //          }
        //          else if (decc < 0)
        //          {

        //              if (acc_actuel_gauche > 0)
        //              {
        //                  acc_actuel_gauche = 0;
        //              }
        //          }

        //          consigne_vit_gauche = Vrob - acc_actuel_gauche * Te;

        //          if (consigne_vit_gauche > Vmax_consigne)
        //          {
        //              consigne_vit_gauche = Vmax_consigne;
        //          }
        //          else if (consigne_vit_gauche < -Vmax_consigne)
        //          {
        //              consigne_vit_gauche = Vmax_consigne;
        //          }

        //          consigne_dist_gauche = odo_tick_gauche + consigne_vit_gauche * Te;*/

        //         // stop_motors();

        //         // freinage_moteur_gauche(true, Vmax_consigne);

        //         // if (((cons - odo_tick_gauche) < limit_reprise_asser) || ((cons - odo_tick_gauche) > -limit_reprise_asser))
        //         // {
        //         //     etat_actuel_vit_roue_folle_gauche = ETAT_ARRET_Vitesse_ROUE_FOLLE_GAUCHE;
        //         // }
        //         /*
        //                 if (consigne_vit_gauche > 0)
        //                 {
        //                     // Réduire la vitesse rapidement avec le freinage moteur
        //                     freinage_moteur_gauche(true, consigne_vit_gauche);

        //                     // Réduire progressivement la consigne de vitesse
        //                     consigne_vit_gauche -= fabs(decc * Te);

        //                     if (consigne_vit_gauche < 0)
        //                     {
        //                         consigne_vit_gauche = 0; // Empêcher des vitesses négatives
        //                     }
        //                 }
        //         */
        //         // Si la vitesse est positive, on applique un freinage actif classique
        //         if (consigne_vit_gauche > 0)
        //         {
        //             // Réduire la vitesse rapidement avec le freinage moteur
        //             freinage_moteur_gauche(true, consigne_vit_gauche);

        //             // Réduire progressivement la consigne de vitesse
        //             consigne_vit_gauche -= fabs(decc * Te);

        //             // Empêcher des vitesses négatives
        //             if (consigne_vit_gauche < 0)
        //             {
        //                 consigne_vit_gauche = 0;
        //             }
        //         }
        //         // Si la vitesse est négative (freinage inverse), appliquer le freinage inverse
        //         else if (consigne_vit_gauche < 0)
        //         {
        //             // Freinage inverse pour ralentir la vitesse négative
        //             freinage_moteur_gauche(false, fabs(consigne_vit_gauche));

        //             // Augmenter la vitesse négative progressivement (décélération inverse)
        //             consigne_vit_gauche += fabs(decc * Te);

        //             // S'assurer que la vitesse ne dépasse pas la valeur maximale en sens inverse
        //             if (consigne_vit_gauche > 0)
        //             {
        //                 consigne_vit_gauche = 0; // Arrêter complètement si la vitesse atteint 0
        //             }
        //         }

        //         // Mise à jour de la consigne de distance
        //         consigne_dist_gauche = odo_tick_gauche + consigne_vit_gauche * Te;

        //         // Vérifier si on est assez proche de la destination pour arrêter
        //         if (((cons - odo_tick_gauche) < limit_reprise_asser) || ((cons - odo_tick_gauche) > -limit_reprise_asser))
        //         {
        //             stop_moteur_gauche();
        //             etat_actuel_vit_roue_folle_gauche = ETAT_ARRET_Vitesse_ROUE_FOLLE_GAUCHE;
        //         }

        //         break;

        //     case ETAT_ARRET_Vitesse_ROUE_FOLLE_GAUCHE:
        //         if (type_ligne_droite)
        //         {
        //             consigne_dist_gauche = cons * 1 / facteur_ajustement_consigne;
        //         }
        //         T_counter_attente_gauche++;
        //         // if (T_counter_attente_gauche > T_attente_gauche)
        //         consigne_odo_gauche_delta = delta_gauche;
        //         if (consigne_odo_gauche_delta == 0)
        //         {
        //             consigne_dist_gauche = cons;
        //             start_asservissement_roue_gauche = 0;
        //             consigne_odo_gauche_prec = odo_tick_gauche;

        //             // coeff_P_roue_folle_tick_gauche = 7.0 / 2;
        //             // coeff_D_roue_folle_tick_gauche = 0.25 / 2;
        //             // coeff_I_roue_folle_tick_gauche = 0.3 * 2;
        //             etat_actuel_vit_roue_folle_gauche = ETAT_VIDE_Vitesse_ROUE_FOLLE_GAUCHE;
        //         }
        //         else
        //         {
        //             stop_moteur_gauche();
        //             consigne_dist_gauche = odo_tick_gauche + 0 * Te;
        //             // coeff_P_roue_folle_tick_gauche = 7.0 / 50;
        //             // coeff_D_roue_folle_tick_gauche = 0;
        //             // coeff_I_roue_folle_tick_gauche = 0.3 * 2;
        //         }
        //         // }
        //         // else
        //         // {
        //         //     etat_actuel_vit_roue_folle_droite = ETAT_VIDE_Vitesse_ROUE_FOLLE_DROITE;
        //         // }
        //         break;

        //     case ETAT_VIDE_Vitesse_ROUE_FOLLE_GAUCHE:
        //         etat_actuel_vit_roue_folle_gauche = ETAT_ATTENTE_Vitesse_ROUE_FOLLE_GAUCHE;

        //         break;
        //     }

        //     return consigne_dist_gauche;
        // }

        // */

/*
double regulation_vitesse_roue_folle_gauche(float cons, float Vmax_consigne)
{
    if (type_ligne_droite)
    {
        cons = cons * facteur_ajustement_consigne;
    }

    float erreur_vit;
    // if (fabs(cons) < 4096.0)
    // {
    //     if (Vmax_consigne > 115)
    //     {
    //         Vmax_consigne = 115;
    //     }
    // }
    float coeff = 1;
    float vit = Vmax_consigne * coeff;
    float accel = Amax * coeff;
    float decc = Dmax * coeff;
    double Vrob = (delta_gauche) / Te;

    float Ta = vit / accel;
    float Td = vit / decc;
    float Tc = (2.0 * cons - accel * (Ta * Ta + Td * Td)) / (2 * vit);
    distance_accel_gauche = 0.5 * Ta * Ta * accel;
    distance_decl_gauche = 0.5 * Td * Td * decc;

    erreur_vit = vit - (Vrob * Vmax);
    somme_erreur_vit_roue_folle_gauche += erreur_vit * Te;

    if (somme_erreur_vit_roue_folle_gauche > integral_limit)
    {
        somme_erreur_vit_roue_folle_gauche = integral_limit;
    }
    else if (somme_erreur_vit_roue_folle_gauche < -integral_limit)
    {
        somme_erreur_vit_roue_folle_gauche = -integral_limit;
    }

    float derivee_erreur_vit = (erreur_vit - erreur_vit_precedente_roue_folle_gauche) / Te;
    erreur_vit_precedente_roue_folle_gauche = erreur_vit;
    float commande_vit = kp_vit * erreur_vit + ki_vit * somme_erreur_vit_roue_folle_gauche + kd_vit * derivee_erreur_vit;
    // Serial.printf("CmdVit %4.3f ", commande_vit);
    switch (etat_actuel_vit_roue_folle_gauche)
    {
    case ETAT_ACCELERATION_Vitesse_ROUE_FOLLE_GAUCHE:
        acc_actuel_gauche = acc_actuel_gauche + commande_vit * Te;
        // Serial.printf(" Acc %4.4f ", acc_actuel_gauche);

        if (acc_actuel_gauche > accel)
        {
            acc_actuel_gauche = accel;
        }
        else if (acc_actuel_gauche < -accel)
        {
            acc_actuel_gauche = -accel;
        }
        // Serial.printf(" Accpre %4.4f ", acc_actuel_gauche);

        consigne_vit_gauche = Vrob + acc_actuel_gauche * Te;
        // Serial.printf("CsdVitG %4.4f Vrob %.3f acc_actuel_gauche * Te %.0f", consigne_vit_gauche, Vrob, (acc_actuel_gauche * Te));

        if (consigne_vit_gauche > Vmax_consigne)
        {
            consigne_vit_gauche = Vmax_consigne;
        }
        if (consigne_vit_gauche < -Vmax_consigne)
        {
            consigne_vit_gauche = Vmax_consigne;
        }
        // Serial.printf("CsdVitGPre %4.4f ", consigne_vit_gauche);

        consigne_dist_gauche = odo_tick_gauche + consigne_vit_gauche * Te;
        // Serial.printf(" CsDistG %4.4f ", consigne_dist_gauche);

        // Serial.println();
        // Serial.println();

        Ta_counter_gauche++;
        if (Ta_counter_gauche >= fabs(Ta))
        {
            etat_actuel_vit_roue_folle_gauche = ETAT_CROISIERE_Vitesse_ROUE_FOLLE_GAUCHE;
        }
        // Serial.printf("ACCELERATION|");
        break;

    case ETAT_CROISIERE_Vitesse_ROUE_FOLLE_GAUCHE:
        consigne_vit_gauche = vit;

        if (consigne_vit_gauche > Vmax_consigne)
        {
            consigne_vit_gauche = Vmax_consigne;
        }
        else if (consigne_vit_gauche < -Vmax_consigne)
        {
            consigne_vit_gauche = Vmax_consigne;
        }

        consigne_dist_gauche = odo_tick_gauche + consigne_vit_gauche * Te;
        if (fabs(cons - odo_tick_gauche) < fabs(distance_decl_gauche))
        {
            etat_actuel_vit_roue_folle_gauche = ETAT_DECELERATION_Vitesse_ROUE_FOLLE_GAUCHE;
        }

        break;

    case ETAT_DECELERATION_Vitesse_ROUE_FOLLE_GAUCHE:
        acc_actuel_gauche = acc_actuel_gauche - commande_vit * Te;
        if (decc > 0)
        {
            if (acc_actuel_gauche < 0)
            {
                acc_actuel_gauche = 0;
            }
        }
        else if (decc < 0)
        {

            if (acc_actuel_gauche > 0)
            {
                acc_actuel_gauche = 0;
            }
        }

        consigne_vit_gauche = Vrob - acc_actuel_gauche * Te;
        if (consigne_vit_gauche > Vmax_consigne)
        {
            consigne_vit_gauche = Vmax_consigne;
        }
        else if (consigne_vit_gauche < -Vmax_consigne)
        {
            consigne_vit_gauche = Vmax_consigne;
        }

        consigne_dist_gauche = odo_tick_gauche + consigne_vit_gauche * Te;
        if (((cons - odo_tick_gauche) < limit_reprise_asser) || ((cons - odo_tick_gauche) > -limit_reprise_asser))
        {
            etat_actuel_vit_roue_folle_gauche = ETAT_ARRET_Vitesse_ROUE_FOLLE_GAUCHE;
        }
        // stop_motors();
        // Serial.printf("DECELERATION|");
        break;

    case ETAT_ARRET_Vitesse_ROUE_FOLLE_GAUCHE:
        if (type_ligne_droite)
        {
            consigne_dist_gauche = cons * 1 / facteur_ajustement_consigne;

            T_counter_attente_gauche++;
            if (T_counter_attente_gauche > T_attente_gauche)
            {
                coeff_P_roue_folle_tick_gauche = 7.0 / 2;
                coeff_D_roue_folle_tick_gauche = 0.25 / 2;
                coeff_I_roue_folle_tick_gauche = 0.3 * 2;
            }
            else
            {
                coeff_P_roue_folle_tick_gauche = 7.0 / 50;
                coeff_D_roue_folle_tick_gauche = 0;
                coeff_I_roue_folle_tick_gauche = 0.3 * 2;
            }
        }
        else
        {
            consigne_dist_gauche = cons;
        }

        // stop_motors();
        // Serial.printf("ARRÊT atteint|");
        break;
    }

    // Serial.printf("accactu %.2f | Vrob %.5f | ConsVit %3.0f | ConsDit %4.0f | err %.0f | distdeccel %.2f | Decl %f | odo %f | rest %f ",
    //               accel, Vrob, consigne_vit_gauche, consigne_dist_gauche, erreur_test, distance_decl_gauche, decc, odo_tick_gauche, (cons - odo_tick_gauche));
    // Serial.printf("accactu %.2f | Vrob %.5f | ConsVit %3.0f | ConsDit %4.0f | err %.0f | distdeccel %.2f | Decl %f | odo %f | rest %f ",
    //               acc_actuel_gauche, Vrob, consigne_vit_gauche, consigne_dist_gauche, erreur_test, distance_decl_gauche, decc, odo_tick_gauche, (cons - odo_tick_gauche));

    // Serial.printf("Cons %4.0f Vmax %3.0f CsdistG %.0f CsVitG %4.2f rest%f ", cons, Vmax_consigne, consigne_dist_gauche, consigne_vit_gauche, (cons - odo_tick_gauche));
    // Serial.printf("return %4.4f", consigne_dist_gauche);
    return consigne_dist_gauche;
}

*/