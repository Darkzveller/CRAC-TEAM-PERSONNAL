#include "Variable.h"
#include "ASSERVISSEMENT.h"
#include "MOUVEMENT.h"

extern float consigne_odo_droite_prec;
extern float consigne_odo_gauche_prec;
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

    Serial.printf(" vitesse_croisiere_gauche %.0f ", vitesse_croisiere_gauche);
    Serial.printf(" vitesse_croisiere_droit %.0f ", vitesse_croisiere_droit);

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
    type_ligne_droite = true;
    float vitesse_croisiere_gauche = vitesse * sens;
    float vitesse_croisiere_droit = vitesse * sens;

    int consigne_gauche = consigne * sens;
    int consigne_droite = consigne * sens;
    consigne_regulation_vitesse_droite = regulation_vitesse_roue_folle_droite((consigne_droite), vitesse_croisiere_droit);
    consigne_regulation_vitesse_gauche = regulation_vitesse_roue_folle_gauche((consigne_gauche), vitesse_croisiere_gauche);

    float kp_angle_correction = 0.5;

    float erreur_angle_correction = consigne_regulation_vitesse_droite - consigne_regulation_vitesse_gauche;
    float correction = kp_angle_correction * erreur_angle_correction;

    consigne_regulation_vitesse_droite -= correction;
    consigne_regulation_vitesse_gauche += correction;
}
