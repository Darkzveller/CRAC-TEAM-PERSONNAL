#include "Variable.h" 
#include "ASSERVISSEMENT.h"
#include "MOUVEMENT.h"


void rotation(int consigne, int vitesse, int sens)
{
    type_ligne_droite = false;

    float vitesse_croisiere_gauche = vitesse * sens;
    float vitesse_croisiere_droit = vitesse * -sens;

    int consigne_gauche = consigne * sens;
    int consigne_droite = consigne * -sens;
    // if (consigne_droite > 0)
    // {
    //     vitesse_croisiere_droit = vitesse_croisiere_droit;
    // }
    // else if (consigne_droite < 0)
    // {
    //     vitesse_croisiere_droit = -vitesse_croisiere_droit;
    // }
    // if (consigne_gauche > 0)
    // {
    //     vitesse_croisiere_gauche = vitesse_croisiere_gauche;
    // }
    // else if (consigne_gauche < 0)
    // {
    //     vitesse_croisiere_gauche = -vitesse_croisiere_gauche;
    // }
    // Calcul initial des consignes de vitesse pour chaque roue
    consigne_regulation_vitesse_droite = regulation_vitesse_roue_folle_droite(consigne_droite, vitesse_croisiere_droit);
    consigne_regulation_vitesse_gauche = regulation_vitesse_roue_folle_gauche(consigne_gauche, vitesse_croisiere_gauche);

    // Imposer une symétrie des consignes de vitesse
    float vitesse_moyenne = (consigne_regulation_vitesse_droite - consigne_regulation_vitesse_gauche) / 2;

    // On force les consignes à être égales et opposées
    consigne_regulation_vitesse_droite = vitesse_moyenne;
    consigne_regulation_vitesse_gauche = -vitesse_moyenne;
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

