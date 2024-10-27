#include "OTA.h" // Inclusion de la bibliothèque OTA (Over-The-Air) pour les mises à jour sans fil
#include "Variable.h"
#include "MOTEUR.h"
#include "EncoderManager.h"
#include "ASSERVISSEMENT.h"
#include <mat.h>

float f = 0;

float vitesse_roue_droite_actuelle = 0;

// float Vmax = 0.1;
// float Amax = 0.2;
// float Afrein = 1000;
// float acc, vcc;

// float dist = 100;

// float Vmax = 30.0 * 0.10; // Vitesse maximale en m/s
// float coeff_amax = 0.01;
// float Amax = 30* coeff_amax; // Accélération maximale en m/s² (rampe douce)
float Vmax = 250; // Vitesse maximale en m/s
float coeff_amax = 0.01;
float Amax = 50; // Accélération maximale en m/s² (rampe douce)
float Dmax = 50;
float pouravoir = 0.1 / coeff_amax;
float Afrein = 100; // Accélération de freinage en m/s² (freinage rapide)
float dist = 4096;  // Distance en ticks (ajuster selon tes besoins)

float acc_actuel;
float distanceActu;
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
double regulation_vitesse(float cons)
{
    float coeff = 1;
    float vit = Vmax * coeff;
    float accel = Amax * coeff;
    float decc = Dmax * coeff;

    double Vrob = (delta_droit) / Te; // Calcul de la vitesse actuelle en
    if (stop == 0)
    {

        Ta = (vit / accel);
        // Ta = 200;
        Td = (vit / decc);
        // Td = Vmax / Amax;
        Tc = (2.0 * cons - accel * (Ta * Ta + Td * Td)) / (2 * vit);
        distance_accel = 0.5 * Ta * Ta * accel;
        distance_decl = 0.5 * Td * Td * decc;
        // distance_decl = cons -distance_accel;
        if (Ta_counter <= Ta)
        {
            Serial.printf("acccccccc ");
            Ta_counter++;
            acc_actuel++;
            if (acc_actuel >= accel)
            {
                acc_actuel = accel;
            }

            consigne_vit += (Vrob + acc_actuel * Te);
            consigne_dist = odo_tick_droit + consigne_vit * Te;
        }
        else if ((cons - odo_tick_droit) < (distance_decl))
        {
            Serial.printf("decce ");
            acc_actuel--;
            if (acc_actuel <= 1)
            {
                acc_actuel = 0;
                stop = 1;
            }
            consigne_vit = Vrob - acc_actuel * Te;
            consigne_dist = odo_tick_droit + consigne_vit * Te;
        }
        else
        {
            Serial.printf("Vit ");
            consigne_dist = odo_tick_droit + consigne_vit * Te;
            Tc_counter++;
        }
    }
    // Serial.printf("accactu %.2f acccalc %.2f ConsVit %4.0f ConsDit %4.0f err %.0f distdeccel%.2f odo %f Ta%.2f Tacount %f Tc%.2f pouravoir %2.0f ", accel, acc_actuel, consigne_vit, consigne_dist, erreur_test, distance_decl, odo_tick_droit, Ta, Ta_counter, Tc, pouravoir);
    Serial.printf("accactu %.2f acccalc %.2f ConsVit %4.0f ConsDit %4.0f err %.0f distdeccel%.2f Decl %f odo %f ", accel, acc_actuel, consigne_vit, consigne_dist, erreur_test, distance_decl, decc, odo_tick_droit);

    // Serial.println();

    // delay(500);
    return consigne_dist;
}

/**
double regulation_vitesse(int choice_mode)
{
    double Vdist;                                          // Consigne de vitesse en ticks/s
    double Vrob = (delta_droit + delta_gauche) / 2.0 / Te; // Calcul de la vitesse actuelle en ticks/s
    double Dfrein = Vrob * Vrob / (2 * Afrein);            // Distance de freinage en ticks

    // Condition de freinage
    if (choice_mode == 0)
    {
        if (dist < Dfrein) // Phase de freinage
        {
            Serial.printf("Frein ");
            Vdist = Vrob - (Afrein * Te); // Ralentir progressivement avec Afrein
        }
        else if (Vrob < Vmax) // Phase d'accélération
        {
            Serial.printf("Acc ");
            Vdist = Vrob + (Amax * Te); // Augmenter la vitesse progressivement avec Amax
        }
        else // Maintien de la vitesse maximale
        {
            Serial.printf("Vit ");
            Vdist = Vmax; // Maintien de la vitesse maximale
        }
    }

    // Calcul de la consigne en ticks sur la période d'échantillonnage
    double consigne_ticks = Vdist * Te; // Convertir la vitesse en ticks/s en nombre de ticks sur la période Te
    Serial.printf("Consigne en ticks: %5.2f ", consigne_ticks);
    return consigne_ticks; // Retourner la consigne en ticks
}
*/

void controle(void *parameters)
{
    TickType_t xLastWakeTime;
    xLastWakeTime = xTaskGetTickCount();
    while (1)
    {
        // f = Vmax * Te + 0;
        // Serial.printf("D = %4.6f mm ", f);

        // f = (f * TIC_PER_TOUR) / (2.0 * M_PI * (SIZE_WHEEL_DIAMETER_mm / 2.0));

        // vitesse_roue_droite_actuelle = delta_droit / Te;
        // Serial.printf("f = %4.6f tick ", f);
        // Serial.printf("Vitesse droit = %4.6f mm/ms", vitesse_roue_droite_actuelle);
        // Serial.println();
        // asservissement_roue_folle_droite_tick(f, odo_tick_droit);
        f = regulation_vitesse(dist);

        if ((flag_controle = 1) == 1)
        {
            asservissement_roue_folle_droite_tick(f, odo_tick_droit);
            // asservissement_roue_folle_gauche_tick(f, odo_tick_gauche);
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
