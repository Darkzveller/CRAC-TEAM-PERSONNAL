#include "OTA.h" // Inclusion de la bibliothèque OTA (Over-The-Air) pour les mises à jour sans fil
#include "MOTEUR.h"
void setup()
{
    setup_motors();
    // Initialisation de la communication série à 115200 bauds
    Serial.begin(115200);
    Serial.println("Booting with OTA"); // Message indiquant le démarrage avec OTA
    // Appel à la fonction de configuration OTA (non définie dans ce code, mais probablement ailleurs)
    // setupOTA();

    // Boucle jusqu'à ce qu'un client soit connecté via le port série WiFi
    // while (!SerialWIFI.available())
    // {
    //     delay(500);                                             // Attente de 500 ms avant de vérifier à nouveau
    //     Serial.println("Aucun client connecté, en attente..."); // Message indiquant qu'il n'y a pas de client connecté
    // }
    // affichage_commande_wifi();
    Serial.println("on commence");
}

// Boucle principale, exécutée en continu après le setup
void loop()
{
    Serial.println("Un tour de boucle effectuer");
    int temps = 2500;
    int pwm = 1024;
    Serial.println("Premier test");

    for (int jsp = 0; jsp < 4096; jsp++)
    {
        Serial.printf("pwm = %d\n", jsp);
        moteur_droit(jsp, true);
        moteur_gauche(jsp, true);
        delay(10);
    }

    for (int jsp = 4096; jsp > 1; jsp--)
    {
        Serial.printf("pwm = %d\n", jsp);
        moteur_droit(jsp, true);
        moteur_gauche(jsp, true);
        delay(10);
    }

    delay(temps);

    for (int jsp = 0; jsp < 4096; jsp++)
    {
        Serial.printf("pwm = %d\n", jsp);
        moteur_droit(jsp, false);
        moteur_gauche(jsp, false);
        delay(10);
    }

    for (int jsp = 4096; jsp > 1; jsp--)
    {
        Serial.printf("pwm = %d\n", jsp);
        moteur_droit(jsp, false);
        moteur_gauche(jsp, false);
        delay(10);
    }
}
