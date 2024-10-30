#include "OTA.h" // Inclusion de la bibliothèque OTA (Over-The-Air) pour les mises à jour sans fil
#include "Variable.h"
#include "MOTEUR.h"
#include "EncoderManager.h"
#include "ASSERVISSEMENT.h"
#include <mat.h>

float f = 0;
float e = 0;
float Amax = 50; // Accélération maximale en m/s² (rampe douce)
float Dmax = 7.5;
float avncement_droite = 3000 * 1; // Distance en ticks (ajuster selon tes besoins)
float avncement_gauche = 3000 * 1; // Distance en ticks (ajuster selon tes besoins)

float acc_actuel_droite;
double consigne_vit_droite;
double consigne_dist_droite;

float acc_actuel_gauche;
double consigne_vit_gauche;
double consigne_dist_gauche;

double Ta_counter_droite = 0;
double Ta_counter_gauche = 0;

double Td_counter_droite = 0;
double Td_counter_gauche = 0;

bool stop = 0;

float distance_accel_droite = 0;
float distance_decl_droite = 0;

float distance_accel_gauche = 0;
float distance_decl_gauche = 0;

float Vmax = 145; // Limite de vitesse maximale souhaitée

float kp_vit = 2.5, ki_vit = 0.1, kd_vit = 0.05;
float erreur_vit_precedente_roue_folle_droite = 0;
float integral_limit = 500;
float somme_erreur_vit_roue_folle_droite = 0;

float somme_erreur_vit_roue_folle_gauche = 0;
float erreur_vit_precedente_roue_folle_gauche = 0;

// Définition des constantes et des variables d'état
enum Etat_vitesse_roue_folle_droite
{
    ETAT_ACCELERATION_Vitesse_ROUE_FOLLE_DROITE,
    ETAT_CROISIERE_Vitesse_ROUE_FOLLE_DROITE,
    ETAT_DECELERATION_Vitesse_ROUE_FOLLE_DROITE,
    ETAT_ARRET_Vitesse_ROUE_FOLLE_DROITE
};
Etat_vitesse_roue_folle_droite etat_actuel_vit_roue_folle_droite = ETAT_ACCELERATION_Vitesse_ROUE_FOLLE_DROITE;
// Définition des constantes et des variables d'état
enum Etat_vitesse_roue_folle_gauche
{
    ETAT_ACCELERATION_Vitesse_ROUE_FOLLE_GAUCHE,
    ETAT_CROISIERE_Vitesse_ROUE_FOLLE_GAUCHE,
    ETAT_DECELERATION_Vitesse_ROUE_FOLLE_GAUCHE,
    ETAT_ARRET_Vitesse_ROUE_FOLLE_GAUCHE
};
Etat_vitesse_roue_folle_gauche etat_actuel_vit_roue_folle_gauche = ETAT_ACCELERATION_Vitesse_ROUE_FOLLE_GAUCHE;

double regulation_vitesse_roue_folle_droite(float cons, float Vmax_consigne)
{

    float erreur_vit;
    if (cons < 4096.0)
    {
        if (Vmax_consigne > 115)
        {
            Vmax_consigne = 115;
        }
    }
    float coeff = 1;
    float vit = Vmax_consigne * coeff;
    float accel = Amax * coeff;
    float decc = Dmax * coeff;
    double Vrob = (delta_droit) / Te; // Calcul de la vitesse actuelle

    // Calcul des distances et des temps
    float Ta = vit / accel;
    float Td = vit / decc;
    float Tc = (2.0 * cons - accel * (Ta * Ta + Td * Td)) / (2 * vit);
    distance_accel_droite = 0.5 * Ta * Ta * accel;
    distance_decl_droite = 0.5 * Td * Td * decc;

    // Calcul PID de la vitesse
    erreur_vit = vit - (Vrob * Vmax);
    somme_erreur_vit_roue_folle_droite += erreur_vit * Te;

    // Limite pour somme_erreur_vit
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

    // Machine à états
    switch (etat_actuel_vit_roue_folle_droite)
    {
    case ETAT_ACCELERATION_Vitesse_ROUE_FOLLE_DROITE:
        acc_actuel_droite = acc_actuel_droite + commande_vit * Te;
        // Limite pour acc_actuel_droite
        if (acc_actuel_droite > accel)
        {
            acc_actuel_droite = accel;
        }

        consigne_vit_droite = Vrob + acc_actuel_droite * Te;
        // Limite pour consigne_vit_droite
        if (consigne_vit_droite > Vmax_consigne)
        {
            consigne_vit_droite = Vmax_consigne;
        }

        consigne_dist_droite = odo_tick_droit + consigne_vit_droite * Te;
        Ta_counter_droite++;
        if (Ta_counter_droite >= Ta)
        {
            etat_actuel_vit_roue_folle_droite = ETAT_CROISIERE_Vitesse_ROUE_FOLLE_DROITE;
        }
        // Serial.printf("ACCELERATION|");
        break;

    case ETAT_CROISIERE_Vitesse_ROUE_FOLLE_DROITE:
        consigne_vit_droite = vit;
        // Limite pour consigne_vit_droit
        if (consigne_vit_droite > Vmax_consigne)
        {
            consigne_vit_droite = Vmax_consigne;
        }

        consigne_dist_droite = odo_tick_droit + consigne_vit_droite * Te;
        if ((cons - odo_tick_droit) < (distance_decl_droite))
        {
            etat_actuel_vit_roue_folle_droite = ETAT_DECELERATION_Vitesse_ROUE_FOLLE_DROITE;
        }
        // Serial.printf("CROISIERE|");
        break;

    case ETAT_DECELERATION_Vitesse_ROUE_FOLLE_DROITE:
        acc_actuel_droite = acc_actuel_droite - commande_vit * Te;
        // Limite pour acc_actuel_droite
        if (acc_actuel_droite < 0)
        {
            acc_actuel_droite = 0;
        }

        consigne_vit_droite = Vrob - acc_actuel_droite * Te;
        // Limite pour consigne_vit_droite
        if (consigne_vit_droite > Vmax_consigne)
        {
            consigne_vit_droite = Vmax_consigne;
        }

        consigne_dist_droite = odo_tick_droit + consigne_vit_droite * Te;
        if ((cons - odo_tick_droit) < 1.0)
        {
            etat_actuel_vit_roue_folle_droite = ETAT_ARRET_Vitesse_ROUE_FOLLE_DROITE;
        }
        // Serial.printf("DECELERATION|");
        break;

    case ETAT_ARRET_Vitesse_ROUE_FOLLE_DROITE:
        consigne_dist_droite = cons; // Fixe la consigne de distance finale
        // stop_motors();        // Fonction d'arrêt des moteurs
        // Serial.printf("ARRÊT atteint|");
        break;
    }

    // Affichage des informations de débogage
    // Serial.printf("Etat %d | accactu %.2f | Vrob %.5f | ConsVit %3.0f | ConsDit %4.0f | err %.0f | distdeccel %.2f | Decl %f | odo %f | rest %f ",
    //               etat_actuel_vit_roue_folle_droite, accel, Vrob, consigne_vit_droite, consigne_dist_droite, erreur_test, distance_decl_droite, decc, odo_tick_droit, (cons - odo_tick_droit));

    return consigne_dist_droite;
}

double regulation_vitesse_roue_folle_gauche(float cons, float Vmax_consigne)
{
    float erreur_vit;
    if (cons < 4096.0)
    {
        if (Vmax_consigne > 115)
        {
            Vmax_consigne = 115;
        }
    }
    float coeff = 1;
    float vit = Vmax_consigne * coeff;
    float accel = Amax * coeff;
    float decc = Dmax * coeff;
    double Vrob = (delta_gauche) / Te; // Calcul de la vitesse actuelle

    // Calcul des distances et des temps
    float Ta = vit / accel;
    float Td = vit / decc;
    float Tc = (2.0 * cons - accel * (Ta * Ta + Td * Td)) / (2 * vit);
    distance_accel_gauche = 0.5 * Ta * Ta * accel;
    distance_decl_gauche = 0.5 * Td * Td * decc;

    // Calcul PID de la vitesse
    erreur_vit = vit - (Vrob * Vmax);
    somme_erreur_vit_roue_folle_gauche += erreur_vit * Te;

    // Limite pour somme_erreur_vit
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

    // Machine à états
    switch (etat_actuel_vit_roue_folle_gauche)
    {
    case ETAT_ACCELERATION_Vitesse_ROUE_FOLLE_GAUCHE:
        acc_actuel_gauche = acc_actuel_gauche + commande_vit * Te;
        // Limite pour acc_actuel_droite
        if (acc_actuel_gauche > accel)
        {
            acc_actuel_gauche = accel;
        }

        consigne_vit_gauche = Vrob + acc_actuel_gauche * Te;
        // Limite pour consigne_vit_droite
        if (consigne_vit_gauche > Vmax_consigne)
        {
            consigne_vit_gauche = Vmax_consigne;
        }

        consigne_dist_gauche = odo_tick_gauche + consigne_vit_gauche * Te;
        Ta_counter_gauche++;
        if (Ta_counter_gauche >= Ta)
        {
            etat_actuel_vit_roue_folle_gauche = ETAT_CROISIERE_Vitesse_ROUE_FOLLE_GAUCHE;
        }
        // Serial.printf("ACCELERATION|");
        break;

    case ETAT_CROISIERE_Vitesse_ROUE_FOLLE_GAUCHE:
        consigne_vit_gauche = vit;
        // Limite pour consigne_vit_droite
        if (consigne_vit_gauche > Vmax_consigne)
        {
            consigne_vit_gauche = Vmax_consigne;
        }

        consigne_dist_gauche = odo_tick_gauche + consigne_vit_gauche * Te;
        if ((cons - odo_tick_gauche) < (distance_decl_gauche))
        {
            etat_actuel_vit_roue_folle_gauche = ETAT_DECELERATION_Vitesse_ROUE_FOLLE_GAUCHE;
        }
        // Serial.printf("CROISIERE|");
        break;

    case ETAT_DECELERATION_Vitesse_ROUE_FOLLE_GAUCHE:
        acc_actuel_gauche = acc_actuel_gauche - commande_vit * Te;
        // Limite pour acc_actuel_droite
        if (acc_actuel_gauche < 0)
        {
            acc_actuel_gauche = 0;
        }

        consigne_vit_gauche = Vrob - acc_actuel_gauche * Te;
        // Limite pour consigne_vit_droite
        if (consigne_vit_gauche > Vmax_consigne)
        {
            consigne_vit_gauche = Vmax_consigne;
        }

        consigne_dist_gauche = odo_tick_gauche + consigne_vit_gauche * Te;
        if ((cons - odo_tick_gauche) < 1.0)
        {
            etat_actuel_vit_roue_folle_gauche = ETAT_ARRET_Vitesse_ROUE_FOLLE_GAUCHE;
        }
        // Serial.printf("DECELERATION|");
        break;

    case ETAT_ARRET_Vitesse_ROUE_FOLLE_GAUCHE:
        consigne_dist_gauche = cons; // Fixe la consigne de distance finale
        // stop_motors();        // Fonction d'arrêt des moteurs
        // Serial.printf("ARRÊT atteint|");
        break;
    }

    // Affichage des informations de débogage
    // Serial.printf("Etat %d | accactu %.2f | Vrob %.5f | ConsVit %3.0f | ConsDit %4.0f | err %.0f | distdeccel %.2f | Decl %f | odo %f | rest %f ",
    //               etat_actuel_vit_roue_folle_gauche, accel, Vrob, consigne_vit_gauche, consigne_dist_gauche, erreur_test, distance_decl_gauche, decc, odo_tick_gauche, (cons - odo_tick_gauche));

    return consigne_dist_gauche;
}

void controle(void *parameters)
{
    TickType_t xLastWakeTime;
    xLastWakeTime = xTaskGetTickCount();
    while (1)
    {

        f = regulation_vitesse_roue_folle_droite((avncement_droite), Vmax);
        e = regulation_vitesse_roue_folle_gauche((avncement_gauche), Vmax);
        if ((flag_controle = 1) == 1)
        {
            asservissement_roue_folle_droite_tick(f, odo_tick_droit);
            asservissement_roue_folle_gauche_tick(e, odo_tick_gauche);
        }
        else
        {
            stop_motors();
        }
        Serial.printf(" odo gauche %f odo droite %f", odo_tick_gauche, odo_tick_droit);
        Serial.printf(" e %f f %f", e, f);
        Serial.println();
        // delay(1000);
        // FlagCalcul = 1;
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(Te));
    }
}
void odo(void *parameters)
{
    TickType_t xLastWakeTime;
    xLastWakeTime = xTaskGetTickCount();
    while (1)
    {

        read_x_y_theta();

        // FlagCalcul = 1;
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(Te));
    }
}

void setup()
{ // calcul coeff filtre
    delay(10000);
    // Initialisation de la communication série à 115200 bauds
    Serial.begin(115200);
    // Serial.println("Booting with OTA"); // Message indiquant le démarrage avec OTA
    // Appel à la fonction de configuration OTA (non définie dans ce code, mais probablement ailleurs)
    setupOTA();
    // Initialisation des moteurs
    setup_motors();
    // Initialisation des encodeurs
    setup_encodeur();

    // Boucle jusqu'à ce qu'un client soit connecté via le port série WiFi
    // while (!SerialWIFI.available())
    // {
    //     delay(500);                                             // Attente de 500 ms avant de vérifier à nouveau
    //     Serial.println("Aucun client connecté, en attente..."); // Message indiquant qu'il n'y a pas de client connecté
    // }
    // delay(10000);
    // affichage_commande_wifi();
    Serial.println("on commence");
    Serial.printf("avncement_gauche enter : %f\n", avncement_gauche);
    Serial.printf("avncement_droite enter : %f\n", avncement_droite);

    delay(5000);
    xTaskCreate(
        controle,   // nom de la fonction
        "controle", // nom de la tache que nous venons de vréer
        10000,      // taille de la pile en octet
        NULL,       // parametre
        10,         // tres haut niveau de priorite
        NULL        // descripteur
    );
    xTaskCreate(
        odo,   // nom de la fonction
        "odo", // nom de la tache que nous venons de vréer
        10000, // taille de la pile en octet
        NULL,  // parametre
        11,    // tres haut niveau de priorite
        NULL   // descripteur
    );
}

// Boucle principale, exécutée en continu après le setup
void loop()
{
}
