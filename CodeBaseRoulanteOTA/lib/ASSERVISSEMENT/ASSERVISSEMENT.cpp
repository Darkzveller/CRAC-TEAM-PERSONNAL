#include "Variable.h" // Remonte d'un niveau pour atteindre lib
#include "MOTEUR.h"
#include "ASSERVISSEMENT.h"
#include "OTA.h"
float facteur_ajustement_consigne = 0.5;

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

    // if ((erreur < 1) || (erreur > -1))
    // {

    //   somme_integral_roue_folle_droite_tick = 0;
    // }
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
    // Serial.printf("cmd %5.2f err %5.2f obs %5.2f  cons %5.2f ", commande,erreur, observation, consigne);

    // Serial.printf("cmd %5.2f P %5.2f D %5.2f I %5.2f Sinte %5.2f err %5.2f obs %5.2f  cons %5.2f resol %4.0f", commande, proportionnel, deriver, integral, somme_integral_roue_folle_droite_tick, erreur, observation, consigne, resolution_calculer);
    // SerialWIFI.printf("cmd %5.2f P %5.2f D %5.2f I %5.2f Sinte %5.2f err %5.2f obs %5.2f  cons %5.2f resol %4.0f", commande, proportionnel, deriver, integral, somme_integral_roue_folle_droite_tick, erreur, observation, consigne, resolution_calculer);
    // SerialWIFI.println();
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

    // Serial.printf("cmd %5.2f P %5.2f D %5.2f I %5.2f Sinte %5.2f err %5.2f obs %5.2f  cons %5.2f ", commande, proportionnel, deriver, integral, somme_integral_roue_folle_tick, erreur, observation, consigne);
    moteur_gauche(sortie_roue_folle, sens);
}

double regulation_vitesse_roue_folle_droite(float cons, float Vmax_consigne)
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
    // Serial.printf("CmdVit %4.3f ", commande_vit);
    switch (etat_actuel_vit_roue_folle_droite)
    {
    case ETAT_ACCELERATION_Vitesse_ROUE_FOLLE_DROITE:
        acc_actuel_droite = acc_actuel_droite + commande_vit * Te;
        // Serial.printf(" Acc %4.4f ", acc_actuel_droite);

        if (acc_actuel_droite > accel)
        {
            acc_actuel_droite = accel;
        }
        else if (acc_actuel_droite < -accel)
        {
            acc_actuel_droite = -accel;
        }
        // Serial.printf(" Accpre %4.4f ", acc_actuel_droite);

        consigne_vit_droite = Vrob + acc_actuel_droite * Te;
        // Serial.printf("CsdVitD %4.4f Vrob %.3f acc_actuel_droite * Te %.0f", consigne_vit_droite, Vrob, (acc_actuel_droite * Te));

        if (consigne_vit_droite > Vmax_consigne)
        {
            consigne_vit_droite = Vmax_consigne;
        }
        if (consigne_vit_droite < -Vmax_consigne)
        {
            consigne_vit_droite = Vmax_consigne;
        }
        // Serial.printf("CsdVitDPre %4.4f ", consigne_vit_droite);

        consigne_dist_droite = odo_tick_droit + consigne_vit_droite * Te;
        // Serial.printf(" CsDistD %4.4f ", consigne_dist_droite);

        // Serial.println();
        // Serial.println();

        Ta_counter_droite++;
        if (Ta_counter_droite >= fabs(Ta))
        {
            etat_actuel_vit_roue_folle_droite = ETAT_CROISIERE_Vitesse_ROUE_FOLLE_DROITE;
        }
        Serial.printf("ACCELERATION|");
        break;

    case ETAT_CROISIERE_Vitesse_ROUE_FOLLE_DROITE:
        consigne_vit_droite = vit;

        if (consigne_vit_droite > Vmax_consigne)
        {
            consigne_vit_droite = Vmax_consigne;
        }
        else if (consigne_vit_droite < -Vmax_consigne)
        {
            consigne_vit_droite = Vmax_consigne;
        }

        consigne_dist_droite = odo_tick_droit + consigne_vit_droite * Te;
        if (fabs(cons - odo_tick_droit) < fabs(distance_decl_droite))
        {
            etat_actuel_vit_roue_folle_droite = ETAT_DECELERATION_Vitesse_ROUE_FOLLE_DROITE;
        }
        Serial.printf("CROISIERE|");
        break;

    case ETAT_DECELERATION_Vitesse_ROUE_FOLLE_DROITE:
        acc_actuel_droite = acc_actuel_droite - commande_vit * Te;
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
        if (((cons - odo_tick_droit) < limit_reprise_asser) || ((cons - odo_tick_droit) > -limit_reprise_asser))
        {
            etat_actuel_vit_roue_folle_droite = ETAT_ARRET_Vitesse_ROUE_FOLLE_DROITE;
        }
        Serial.printf("DECELERATION|");
        // stop_motors();

        break;

    case ETAT_ARRET_Vitesse_ROUE_FOLLE_DROITE:
        if (type_ligne_droite)
        {
            consigne_dist_droite = cons * 1 / facteur_ajustement_consigne;
            T_counter_attente_droite++;
            if (T_counter_attente_droite > T_attente_droite)
            {
                coeff_P_roue_folle_tick_droite = 7.0 / 2;
                coeff_D_roue_folle_tick_droite = 0.25 / 2;
                coeff_I_roue_folle_tick_droite = 0.3 * 2;
                etat_actuel_vit_roue_folle_droite = ETAT_VIDE_Vitesse_ROUE_FOLLE_DROITE;
            }
            else
            {
                coeff_P_roue_folle_tick_droite = 7.0 / 50;
                coeff_D_roue_folle_tick_droite = 0;
                coeff_I_roue_folle_tick_droite = 0.3 * 2;
            }
        }
        else
        {
            consigne_dist_droite = cons;
        }
        // stop_motors();
        Serial.printf("ARRÊT atteint|");
        break;
        }

    // Serial.printf("accactu %.2f | Vrob %.5f | ConsVit %3.0f | ConsDit %4.0f | err %.0f | distdeccel %.2f | Decl %f | odo %f | rest %f ",
    //               accel, Vrob, consigne_vit_droite, consigne_dist_droite, erreur_test, distance_decl_droite, decc, odo_tick_droit, (cons - odo_tick_droit));
    // Serial.printf("accactu %.2f | Vrob %.5f | ConsVit %3.0f | ConsDit %4.0f | distdeccel %.2f | Decl %f | odo %f | rest %f ",
    //               acc_actuel_droite, Vrob, consigne_vit_droite, consigne_dist_droite,  distance_decl_droite, decc, odo_tick_droit, (cons - odo_tick_droit));
    Serial.printf("ConsDit %4.0f | distdeccel %.2f | Decl %f |evo cons-odo %f|Tc %f|", consigne_dist_droite, distance_decl_droite, decc, (cons - odo_tick_droit), Tc);

    // Serial.printf("Cons %4.0f Vmax %3.0f CsdistD %.0f CsVitD %4.2f rest%f ", cons, Vmax_consigne, consigne_dist_droite, consigne_vit_droite, (cons - odo_tick_droit));
    // Serial.printf("return %4.4f", consigne_dist_droite);
    return consigne_dist_droite;
}

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

/*
double regulation_vitesse_roue_folle_gauche(float cons, float Vmax_consigne)
{
    // delay(500);
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
        else if (consigne_vit_gauche < Vmax_consigne)
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
        // Serial.printf("CROISIERE|");
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
        else if (consigne_vit_gauche < Vmax_consigne)
        {
            consigne_vit_gauche = Vmax_consigne;
        }

        consigne_dist_gauche = odo_tick_gauche + consigne_vit_gauche * Te;
        if (((cons - odo_tick_gauche) < 1.0) || ((cons - odo_tick_gauche) > -1.0))
        {
            etat_actuel_vit_roue_folle_gauche = ETAT_ARRET_Vitesse_ROUE_FOLLE_GAUCHE;
        }
        // Serial.printf("DECELERATION|");
        break;

    case ETAT_ARRET_Vitesse_ROUE_FOLLE_GAUCHE:
        consigne_dist_gauche = cons;
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
    // Serial.printf("djk %4.3f ", (cons - odo_tick_gauche));
    return consigne_dist_gauche;
}

*/