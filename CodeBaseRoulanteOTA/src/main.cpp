#include "OTA.h" // Inclusion de la bibliothèque OTA (Over-The-Air) pour les mises à jour sans fil
#include "Variable.h"
#include "MOTEUR.h"
#include "EncoderManager.h"
#include <mat.h>

// #include "ASSERVISSEMENT.h"

void controle(void *parameters)
{
    TickType_t xLastWakeTime;
    xLastWakeTime = xTaskGetTickCount();
    while (1)
    {
        read_x_y_theta();

        // probl();

        // FlagCalcul = 1;
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(Te));
    }
}
void setup()
{ // calcul coeff filtre

    // Initialisation de la communication série à 115200 bauds
    Serial.begin(115200);
    Serial.println("Booting with OTA"); // Message indiquant le démarrage avec OTA
                                        // Appel à la fonction de configuration OTA (non définie dans ce code, mais probablement ailleurs)
    // setupOTA();
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
    // affichage_commande_wifi();
    Serial.println("on commence");
    // delay(10000);

    xTaskCreate(
        controle,   // nom de la fonction
        "controle", // nom de la tache que nous venons de vréer
        10000,      // taille de la pile en octet
        NULL,       // parametre
        10,         // tres haut niveau de priorite
        NULL        // descripteur
    );
    delay(7500);
}

// Boucle principale, exécutée en continu après le setup
void loop()
{
    int temps = 1000;
    static int i = 0;
    int pwm = 2000;
    /*
        if (i == 0)
        {
            moteur_droit(pwm, 0);
            moteur_gauche(pwm, 0);
            delay(temps);
            moteur_droit(0, 0);
            moteur_gauche(0, 0);
            delay(5000);

            moteur_droit(2048, 1);
            moteur_gauche(2048, 0);
            delay(340);
            moteur_droit(pwm, 0);
            moteur_gauche(pwm, 0);
            delay(temps);
            // moteur_droit(pwm, 0);
            // moteur_gauche(pwm, 0);

            //     // moteur_droit(pwm, 1);
            //     // moteur_gauche(pwm, 0);
            //     // delay(1000);
            //     // moteur_droit(pwm, 0);
            //     // moteur_gauche(pwm, 0);
            //     // delay(temps);
        }
        else
        {
            stop_motors();
        }
        i++;*/
    moteur_droit(pwm, 1);
    moteur_gauche(pwm, 0);
}
