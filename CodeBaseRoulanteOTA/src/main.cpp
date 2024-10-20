#include "OTA.h" // Inclusion de la bibliothèque OTA (Over-The-Air) pour les mises à jour sans fil
#include "Variable.h"
#include "MOTEUR.h"
#include "EncoderManager.h"
// #include "ASSERVISSEMENT.h"

float theta_robot_prec = 0;
float theta_robot = 0;
void controle(void *parameters)
{
    TickType_t xLastWakeTime;
    xLastWakeTime = xTaskGetTickCount();
    while (1)
    {
        read_encodeurdroit();
        read_encodeurgauche();
        float distance_parcourue = 0.5 * (odo_droit + odo_gauche);

        theta_robot = (((odo_dist_droit - odo_dist_gauche)) / ENTRAXE);
        theta_robot_prec = 1 / (1 + Tau_odo / Te) * theta_robot + Tau_odo / Te * 1 / (1 + Tau_odo / Te) * theta_robot_prec;

        float odo_x = cos(theta_robot) * distance_parcourue;
        float odo_y = sin(theta_robot) * distance_parcourue;


        Serial.printf("distdroit %4.2f  dist gauche %4.2f ", odo_dist_droit, odo_dist_gauche);
        Serial.printf(" x %4.2f mm y %4.2f mm theta %4.2f rad theta %4.2f deg\n", odo_x, odo_y, theta_robot, theta_robot * 180 / 3.14);

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
    delay(10000);
}

// Boucle principale, exécutée en continu après le setup
void loop()
{
    int temps = 1000;
    static int i = 0;
    int pwm = 1024;

    if (i == 0)
    {
        moteur_droit(pwm, 1);
        moteur_gauche(pwm, 0);
        delay(temps);
        moteur_droit(pwm, 0);
        moteur_gauche(pwm, 0);
        delay(temps);
        // moteur_droit(pwm, 1);
        // moteur_gauche(pwm, 1);
        // delay(temps);
        // moteur_droit(pwm, 0);
        // moteur_gauche(pwm, 0);

        // moteur_droit(pwm, 1);
        // moteur_gauche(pwm, 0);
        // delay(1000);
        // moteur_droit(pwm, 0);
        // moteur_gauche(pwm, 0);
        // delay(temps);
    }
    else
    {
        stop_motors();
    }
    i++;
}
