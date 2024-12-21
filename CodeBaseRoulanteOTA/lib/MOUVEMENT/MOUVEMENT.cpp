#include "Variable.h"
#include "ASSERVISSEMENT.h"
#include "MOUVEMENT.h"
#include "MOTEUR.h"

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
    float vitesse_moyenne = (consigne_regulation_vitesse_droite - consigne_regulation_vitesse_gauche) / 2;

    // On force les consignes à être égales et opposées
    // consigne_regulation_vitesse_droite = -sens * vitesse_moyenne;
    // consigne_regulation_vitesse_gauche = sens * vitesse_moyenne;
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

    float erreur_angle_correction = consigne_regulation_vitesse_droite - consigne_regulation_vitesse_gauche;
    float correction = kp_angle_correction * erreur_angle_correction;

    consigne_regulation_vitesse_droite -= correction;
    consigne_regulation_vitesse_gauche += correction;

    // asservissement_correction_angle(consigne_theta_prec, theta_robot);

    // Serial.printf(" consigne_regulation_vitesse_droite %.0f ", consigne_regulation_vitesse_droite);
    // Serial.printf(" consigne_regulation_vitesse_gauche %.0f ", consigne_regulation_vitesse_gauche);
}

void recalage(int cons_distance_ticks, float SEUIL_RECALAGE, int vitesse_recalage, int sens_recalage)
{
    bool recalage_en_cours;
    // Calculer la distance parcourue en prenant la moyenne des deux roues
    float vitesse_croisiere_gauche = vitesse_recalage * sens_recalage;
    float vitesse_croisiere_droit = vitesse_recalage * sens_recalage;

    consigne_regulation_vitesse_droite = regulation_vitesse_roue_folle_droite(cons_distance_ticks, vitesse_croisiere_droit);
    consigne_regulation_vitesse_gauche = regulation_vitesse_roue_folle_gauche(cons_distance_ticks, vitesse_croisiere_gauche);

    // Vérification si la distance parcourue atteint la consigne
    // if (abs(distance_parcourue) >= cons_distance_ticks)
    // {
    //     recalage_en_cours = false; // Le recalage est terminé
    // }
    // Si l'une des roues dépasse un seuil d'erreur, on arrête le moteur correspondant pour éviter le patinage
    if ((fabs(odo_dist_droit) - fabs(odo_dist_gauche)) > SEUIL_RECALAGE)
    {
        if (fabs(odo_dist_droit) > fabs(odo_dist_gauche))
        {
            stop_moteur_droit();
        }
        else
        {
            stop_moteur_gauche();
        }
    }
    // Serial.printf("OdoGauche: %d, OdoDroite: %d, Distance Parcourue: %d\n", odo_dist_droit, odo_dist_gauche, distance_parcourue);
}