#include "OTA.h" // Inclusion de la bibliothèque OTA (Over-The-Air) pour les mises à jour sans fil
#include "Variable.h"
#include "MOTEUR.h"
#include "EncoderManager.h"
#include "ASSERVISSEMENT.h"
#include <mat.h>

float nbr_tour = 1 * 360 / 90;
float avncement_droite = 2250 * nbr_tour;
float avncement_gauche = -2250 * nbr_tour;
// float avncement_droite = 2238 * nbr_tour;
// float avncement_gauche = -2243 * nbr_tour;

void rotation(int consigne, int vitesse, int sens)
{
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
    consigne_regulation_vitesse_droite = regulation_vitesse_roue_folle_droite((consigne_droite), vitesse_croisiere_droit);
    consigne_regulation_vitesse_gauche = regulation_vitesse_roue_folle_gauche((consigne_gauche), vitesse_croisiere_gauche);
    if ((consigne_regulation_vitesse_droite != consigne_droite) && (consigne_regulation_vitesse_gauche != consigne_gauche))
    {
        consigne_regulation_vitesse_gauche = consigne_regulation_vitesse_gauche - (consigne_regulation_vitesse_gauche + consigne_regulation_vitesse_droite);
    }
}
void ligne_droite(int consigne, int vitesse, int sens)
{
    float vitesse_croisiere_gauche = vitesse * sens;
    float vitesse_croisiere_droit = vitesse * sens;

    int consigne_gauche = consigne * sens;
    int consigne_droite = consigne * sens;
    consigne_regulation_vitesse_droite = regulation_vitesse_roue_folle_droite((consigne_droite), vitesse_croisiere_droit);
    consigne_regulation_vitesse_gauche = regulation_vitesse_roue_folle_gauche((consigne_gauche), vitesse_croisiere_gauche);
}

void controle(void *parameters)
{
    TickType_t xLastWakeTime;
    xLastWakeTime = xTaskGetTickCount();
    while (1)
    {
        rotation((2250 * nbr_tour), 70, 1);
        //ligne_droite((6000*10), 145, 1);
        if ((flag_controle = 1) == 1)
        {
            asservissement_roue_folle_droite_tick(consigne_regulation_vitesse_droite, odo_tick_droit);
            asservissement_roue_folle_gauche_tick(consigne_regulation_vitesse_gauche, odo_tick_gauche);
        }
        else
        {
            stop_motors();
        }
        // Serial.printf("obs %4.0f", observation);
        Serial.printf("| odo gauche %.0f odo droite %.0f", odo_tick_gauche, odo_tick_droit);
        // // // SerialWIFI.printf("|consigne_regulation_vitesse_droite %5.2f consigne_regulation_vitesse_gauche %5.2f ", consigne_regulation_vitesse_droite, consigne_regulation_vitesse_gauche);
        // // Serial.printf("| consigne_regulation_vitesse_droite %.0f consigne_regulation_vitesse_gauche_rec  %.0f consigne_regulation_vitesse_gauche_nonrec  %.0f|", consigne_regulation_vitesse_droite, consigne_regulation_vitesse_gauche, jsp);
        Serial.printf(" Theta %3.1f ", theta_robot * 180 / 3.14);
        Serial.printf("nmbr tour %2.3f", (double)(theta_robot * 180 / M_PI / 360));
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

        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(Te));
    }
}

void setup()
{ // calcul coeff filtre
    // delay(10000);
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
    // Serial.printf("avncement_gauche enter : %.0f\n", avncement_gauche);
    // Serial.printf("avncement_droite enter : %.0f\n", avncement_droite);

    reset_encodeur();
    delay(5);
    reset_encodeur();

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
