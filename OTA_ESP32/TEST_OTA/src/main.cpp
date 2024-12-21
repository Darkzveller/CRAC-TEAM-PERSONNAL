#include "OTA.h" // Inclusion de la bibliothèque OTA (Over-The-Air) pour les mises à jour sans fil

// Fonction d'initialisation, exécutée une seule fois au démarrage
void setup()
{
    // Initialisation de la communication série à 115200 bauds
    Serial.begin(115200);
    Serial.println("Booting with OTA"); // Message indiquant le démarrage avec OTA
    // Appel à la fonction de configuration OTA (non définie dans ce code, mais probablement ailleurs)
    setupOTA();

    // Boucle jusqu'à ce qu'un client soit connecté via le port série WiFi
    while (!SerialWIFI.available())
    {
        delay(500);                                             // Attente de 500 ms avant de vérifier à nouveau
        Serial.println("Aucun client connecté, en attente..."); // Message indiquant qu'il n'y a pas de client connecté
    }
    SerialWIFI.println(""); // Pour qu'on puisse nous, de notré coté voir ce qu'on tape dans l'invite de commandes

    // Une fois qu'un client est connecté, envoie "Coucou" via la connexion série WiFi
    // SerialWIFI.println("Coucou");

    // Ici, vous pouvez ajouter d'autres instructions pour le setup si nécessaire
    // Your setup code
}

// Boucle principale, exécutée en continu après le setup
void loop()
{

}
