#include "Variable.h"
#include "ASSERVISSEMENT.h"
#include "MOUVEMENT.h"

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
    Serial.printf(" consigne_gauche %d ", consigne_gauche);
    Serial.printf(" consigne_droite %d ", consigne_droite);

    // Serial.printf(" vitesse_croisiere_gauche %.0f ", vitesse_croisiere_gauche);
    // Serial.printf(" vitesse_croisiere_droit %.0f ", vitesse_croisiere_droit);

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

    Serial.printf(" consigne_gauche %d ", consigne_gauche);
    Serial.printf(" consigne_droite %d ", consigne_droite);

    // Serial.printf(" consigne_odo_gauche_prec %.0f ", consigne_odo_gauche_prec);
    // Serial.printf(" consigne_odo_droite_prec %.0f ", consigne_odo_droite_prec);

    // Calcul initial des consignes de vitesse pour chaque roue
    consigne_regulation_vitesse_droite = regulation_vitesse_roue_folle_droite(consigne_droite, vitesse_croisiere_droit);
    consigne_regulation_vitesse_gauche = regulation_vitesse_roue_folle_gauche(consigne_gauche, vitesse_croisiere_gauche);

    // asservissement_correction_angle(consigne_theta_prec, theta_robot);

    // Serial.printf(" consigne_regulation_vitesse_droite %.0f ", consigne_regulation_vitesse_droite);
    // Serial.printf(" consigne_regulation_vitesse_gauche %.0f ", consigne_regulation_vitesse_gauche);
}
double coeff_P_correction_angle = 100;
double coeff_D_correction_angle = 0;
double coeff_I_correction_angle = 0;
double erreur_prec_correction_angle = 0;
double somme_integral_correction_angle = 0;
double integral_limit_correction_angle = 700;
void asservissement_correction_angle(double consigne, double observation)
{
    double limite_commande = 700;
    double sortie_consigne_regulation;

    double erreur = consigne - observation;
    double proportionnel = erreur * coeff_P_correction_angle;
    // Serial.printf(" erreur %.4f ", erreur);
    // Serial.printf(" observation %.4f ", observation);
    // Serial.printf(" consigne %.4f ", consigne);

    double deriver = coeff_D_correction_angle * (erreur - erreur_prec_correction_angle) / Te;

    somme_integral_correction_angle += erreur * Te;
    if (somme_integral_correction_angle > integral_limit_correction_angle)
    {
        somme_integral_correction_angle = integral_limit_correction_angle;
    }
    else if (somme_integral_correction_angle < -integral_limit_correction_angle)
    {
        somme_integral_correction_angle = -integral_limit_correction_angle;
    }

    double integral = coeff_I_correction_angle * somme_integral_correction_angle;

    double commande = proportionnel + deriver + integral;

    erreur_prec_correction_angle = erreur;

    // Gestion des bornes de la commande
    if (commande > 0)
    {
        if (commande > limite_commande)
        {
            sortie_consigne_regulation = limite_commande;
        }
        else
        {
            sortie_consigne_regulation = commande;
        }
    }
    else
    {
        if (commande < -limite_commande)
        {

            sortie_consigne_regulation = limite_commande;
        }
        else
        {
            sortie_consigne_regulation = -commande;
        }
    }
    // Serial.printf(" commande %.4f ", commande);

    consigne_regulation_vitesse_droite -= commande;
    consigne_regulation_vitesse_gauche += commande;
}
