#include "OTA.h" // Inclusion de la bibliothèque OTA (Over-The-Air) pour les mises à jour sans fil
#include "Variable.h"
#include "MOTEUR.h"
#include "EncoderManager.h"
#include "ASSERVISSEMENT.h"
#include <mat.h>

float f = 0;

float Amax = 50; // Accélération maximale en m/s² (rampe douce)
float Dmax = 7.5;
float dist = 2048 * 1; // Distance en ticks (ajuster selon tes besoins)

float acc_actuel;
double consigne_vit;
double consigne_dist;
double Ta = 0;
double Ta_counter = 0;
double Td = 0;
double Td_counter = 0;
bool stop = 0;
float distance_accel = 0;
float distance_decl = 0;
float distance_v_cons = 0;
float Tc = 0;
float Tc_counter = 0;
float Vmax_consigne = 145; // Limite de vitesse maximale souhaitée

// Définition des constantes et des variables d'état
enum Etat_vitesse_roue_folle_droite
{
    ETAT_ACCELERATION_Vitesse_ROUE_FOLLE_DROITE,
    ETAT_CROISIERE_Vitesse_ROUE_FOLLE_DROITE,
    ETAT_DECELERATION_Vitesse_ROUE_FOLLE_DROITE,
    ETAT_ARRET_Vitesse_ROUE_FOLLE_DROITE
};
Etat_vitesse_roue_folle_droite etat_actuel = ETAT_ACCELERATION_Vitesse_ROUE_FOLLE_DROITE;

float kp_vit = 2.5, ki_vit = 0.1, kd_vit = 0.05;
float erreur_vit_precedente_roue_folle_droite = 0;
float somme_erreur_vit_roue_folle_droite = 0, derivee_erreur_vit, integral_limit = 500;

double regulation_vitesse_roue_folle_droite(float cons)
{
    float erreur_vit;
    if (cons < 4096.0)
    {
        Vmax_consigne = 115;
    }
    float coeff = 1;
    float vit = Vmax_consigne * coeff;
    float accel = Amax * coeff;
    float decc = Dmax * coeff;
    double Vrob = (delta_droit) / Te; // Calcul de la vitesse actuelle

    // Calcul des distances et des temps
    Ta = vit / accel;
    Td = vit / decc;
    Tc = (2.0 * cons - accel * (Ta * Ta + Td * Td)) / (2 * vit);
    distance_accel = 0.5 * Ta * Ta * accel;
    distance_decl = 0.5 * Td * Td * decc;

    // Calcul PID de la vitesse
    erreur_vit = vit - (Vrob * vit);
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

    derivee_erreur_vit = (erreur_vit - erreur_vit_precedente_roue_folle_droite) / Te;
    erreur_vit_precedente_roue_folle_droite = erreur_vit;
    float commande_vit = kp_vit * erreur_vit + ki_vit * somme_erreur_vit_roue_folle_droite + kd_vit * derivee_erreur_vit;

    // Machine à états
    switch (etat_actuel)
    {
    case ETAT_ACCELERATION_Vitesse_ROUE_FOLLE_DROITE:
        acc_actuel = acc_actuel + commande_vit * Te;
        // Limite pour acc_actuel
        if (acc_actuel > accel)
        {
            acc_actuel = accel;
        }

        consigne_vit = Vrob + acc_actuel * Te;
        // Limite pour consigne_vit
        if (consigne_vit > Vmax_consigne)
        {
            consigne_vit = Vmax_consigne;
        }

        consigne_dist = odo_tick_droit + consigne_vit * Te;
        Ta_counter++;
        if (Ta_counter >= Ta)
        {
            etat_actuel = ETAT_CROISIERE_Vitesse_ROUE_FOLLE_DROITE;
        }
        Serial.printf("ACCELERATION|");
        break;

    case ETAT_CROISIERE_Vitesse_ROUE_FOLLE_DROITE:
        consigne_vit = vit;
        // Limite pour consigne_vit
        if (consigne_vit > Vmax_consigne)
        {
            consigne_vit = Vmax_consigne;
        }

        consigne_dist = odo_tick_droit + consigne_vit * Te;
        if ((cons - odo_tick_droit) < (distance_decl))
        {
            etat_actuel = ETAT_DECELERATION_Vitesse_ROUE_FOLLE_DROITE;
        }
        Serial.printf("CROISIERE|");
        break;

    case ETAT_DECELERATION_Vitesse_ROUE_FOLLE_DROITE:
        acc_actuel = acc_actuel - commande_vit * Te;
        // Limite pour acc_actuel
        if (acc_actuel < 0)
        {
            acc_actuel = 0;
        }

        consigne_vit = Vrob - acc_actuel * Te;
        // Limite pour consigne_vit
        if (consigne_vit > Vmax_consigne)
        {
            consigne_vit = Vmax_consigne;
        }

        consigne_dist = odo_tick_droit + consigne_vit * Te;
        if ((cons - odo_tick_droit) < 1.0)
        {
            etat_actuel = ETAT_ARRET_Vitesse_ROUE_FOLLE_DROITE;
        }
        Serial.printf("DECELERATION|");
        break;

    case ETAT_ARRET_Vitesse_ROUE_FOLLE_DROITE:
        consigne_dist = cons; // Fixe la consigne de distance finale
        // stop_motors();        // Fonction d'arrêt des moteurs
        Serial.printf("ARRÊT atteint|");
        break;
    }

    // Affichage des informations de débogage
    Serial.printf("Etat %d | accactu %.2f | Vrob %.5f | ConsVit %3.0f | ConsDit %4.0f | err %.0f | distdeccel %.2f | Decl %f | odo %f | rest %f ",
                  etat_actuel, accel, Vrob, consigne_vit, consigne_dist, erreur_test, distance_decl, decc, odo_tick_droit, (cons - odo_tick_droit));

    return consigne_dist;
}

void controle(void *parameters)
{
    TickType_t xLastWakeTime;
    xLastWakeTime = xTaskGetTickCount();
    while (1)
    {

        f = regulation_vitesse_roue_folle_droite((dist));

        if ((flag_controle = 1) == 1)
        {
            asservissement_roue_folle_droite_tick(f, odo_tick_droit);
            asservissement_roue_folle_gauche_tick(f, odo_tick_gauche);
        }
        else
        {
            stop_motors();
        }
        Serial.printf(" f %f", f);
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
    Serial.printf("cons enter : %f\n", dist);
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
