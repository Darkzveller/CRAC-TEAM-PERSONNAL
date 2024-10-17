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
    static long i = 0; // Déclaration d'une variable statique qui conserve sa valeur entre les appels

    if (SerialWIFI.available())
    {
        String input = SerialWIFI.readString(); // Lire la chaîne complète
        Serial.print("Reçu: ");
        Serial.println(input); // Afficher ce qui a été reçu

        // Supprimer les espaces ou caractères indésirables (comme le retour à la ligne)
        input.trim(); // Enlève les espaces superflus et les retours à la ligne

        // if (input.equals("stop")) // Vérifier si l'entrée est "stop"
        // {
        //     i = 0; // Remise à zéro de la variable
        //     SerialWIFI.println(i);
        //     Serial.println("Valeur mise à zéro");
        // }
        if (input.equals("STOP")) // Vérifier si l'entrée est "stop"
        {
            // i = 0; // Remise à zéro de la variable
            SerialWIFI.println("STOP BASE ROULANTE");
        }
        if (input.equals("start")) // Vérifier si l'entrée est "start"
        {
            SerialWIFI.println(i++); // Incrémenter i et envoyer
            Serial.println("Incrémenté : " + String(i));
        }
    }
}
