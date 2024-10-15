#include "OTA.h" // Inclusion de la bibliothèque pour gérer l'OTA (Over-The-Air)
// Je laisse le routeur choisir l'adresse IP et
// je fixe le nom de mon réseau Wi-Fi, ce qui simplifie les démarches, notamment lors des débogages
//  Informations de connexion WiFi
const char *name_card_elec = "BaseEsp32";                          // Nom d'hôte de la carte ESP32
const char *ssid = "Freebox-587F87";                               // SSID du réseau WiFi
const char *password = "subcrescat-degend@-parciore@2-adducturos"; // Mot de passe du réseau WiFi

// Fonction pour gérer les opérations OTA dans une tâche séparée
void ota_handle(void *parameter)
{
  TickType_t xLastWakeTime;
  xLastWakeTime = xTaskGetTickCount();
  for (;;)
  {
    // Gestion des opérations OTA (vérifie si une mise à jour est en cours)
    ArduinoOTA.handle();
    vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(3500));
  }
}

// Fonction pour configurer l'OTA
void setupOTA()
{
  // Définit le nom d'hôte pour la carte ESP32 sur le réseau
  WiFi.setHostname(name_card_elec);

  // Passe le WiFi en mode station (client du réseau WiFi)
  WiFi.mode(WIFI_STA);

  // Démarre la connexion WiFi avec les identifiants donnés
  WiFi.begin(ssid, password);
  Serial.println();
  Serial.print("SSID : ");
  Serial.println(ssid);

  Serial.println("Connexion au WiFi en cours...");

  // Boucle jusqu'à ce que la connexion au WiFi soit réussie
  while (WiFi.waitForConnectResult() != WL_CONNECTED)
  {
    Serial.println("Connection Failed! Rebooting...");
    delay(1000);
    // Redémarrage si la connexion échoue (commenté pour l'instant)
    // ESP.restart();
  }

  // Affiche le succès de la connexion
  Serial.println("");
  Serial.println("Connexion établie !");

  // Définition du port OTA (par défaut, il est à 3232)
  // ArduinoOTA.setPort(3232);

  // Définit le nom d'hôte pour les opérations OTA
  ArduinoOTA.setHostname(name_card_elec);

  // Pas de mot de passe par défaut pour l'OTA
  // ArduinoOTA.setPassword("amdin");

  // Possibilité de définir un mot de passe via son hachage MD5
  // ArduinoOTA.setPasswordHash("admin");

  /*Ancien commentaire de la partie ArduinoOTA qui était le suivant :
    // Port defaults to 3232
    // ArduinoOTA.setPort(3232);

    // Hostname defaults to esp3232-[MAC]
    ArduinoOTA.setHostname(name_card_elec);

    // No authentication by default
    // ArduinoOTA.setPassword("admin");

    // Password can be set with it's md5 value as well
    // MD5(admin) = admin;
    // ArduinoOTA.setPasswordHash("admin");

  */
  // Configuration des différents événements de l'OTA
  ArduinoOTA
      .onStart([]() // Quand une mise à jour démarre
               {
      String type;
      // Si le type de mise à jour est pour le sketch (programme) ou SPIFFS (système de fichiers)
      if (ArduinoOTA.getCommand() == U_FLASH)
        type = "sketch";
      else // U_SPIFFS
        type = "filesystem";

      // Si on met à jour SPIFFS
      Serial.println("Start updating " + type); })
      .onEnd([]() // Quand la mise à jour est terminée
             { Serial.println("\nEnd"); })
      .onProgress([](unsigned int progress, unsigned int total) // Affiche le progrès de la mise à jour
                  { Serial.printf("Progress: %u%%\r", (progress / (total / 100))); })
      .onError([](ota_error_t error) // Gestion des erreurs lors de la mise à jour OTA
               {
      Serial.printf("Error[%u]: ", error);
      // Affiche un message d'erreur spécifique en fonction du type d'erreur
      if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
      else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
      else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
      else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
      else if (error == OTA_END_ERROR) Serial.println("End Failed"); });

  // Démarre le service OTA
  ArduinoOTA.begin();

  // Démarre la communication série WiFi
  SerialWIFI.begin();

  // Affiche les informations une fois prêtes
  Serial.println("Ready");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP()); // Affiche l'adresse IP attribuée
  Serial.print("Wifi Hostname: ");
  Serial.println(WiFi.getHostname()); // Affiche le nom d'hôte du WiFi
  Serial.print("Arduino Hostname: ");
  Serial.print(ArduinoOTA.getHostname()); // Affiche le nom d'hôte pour OTA
  Serial.println(".local");
  Serial.println("Commande a tapé dans le terminal");
  Serial.print("telnet ");
  Serial.print(ArduinoOTA.getHostname());
  Serial.println(".local 23");

  // Création d'une tâche FreeRTOS pour gérer l'OTA dans un thread séparé
  xTaskCreate(
      ota_handle,   /* Fonction de la tâche à exécuter */
      "OTA_HANDLE", /* Nom de la tâche */
      10000,        /* Taille de la pile en octets */
      NULL,         /* Paramètre passé à la tâche (aucun ici) */
      1,            /* Priorité de la tâche */
      NULL);        /* Handle de la tâche (aucun ici) */
}

// AU cas ou j'ai mal effectué mes commentaires
/*#include "OTA.h"

const char *name_card_elec = "BaseEsp32";
const char *ssid = "Me voici";
const char *password = "youssef13";

void ota_handle(void *parameter)
{
  for (;;)
  {
    ArduinoOTA.handle();
    delay(3500);
  }
}
void setupOTA()
{
  WiFi.setHostname(name_card_elec);

  WiFi.mode(WIFI_STA);

  WiFi.begin(ssid, password);
  Serial.println("Connexion au WiFi en cours...");

  while (WiFi.waitForConnectResult() != WL_CONNECTED)
  {
    Serial.println("Connection Failed! Rebooting...");
    delay(1000);
    // ESP.restart();
  }
  // Affiche le succès de la connexion
  Serial.println("");
  Serial.println("Connexion établie !");
  // Port defaults to 3232
  // ArduinoOTA.setPort(3232);

  // Hostname defaults to esp3232-[MAC]
  ArduinoOTA.setHostname(name_card_elec);

  // No authentication by default
  // ArduinoOTA.setPassword("admin");

  // Password can be set with it's md5 value as well
  // MD5(admin) = admin;
  // ArduinoOTA.setPasswordHash("admin");

  ArduinoOTA
      .onStart([]()
               {
      String type;
      if (ArduinoOTA.getCommand() == U_FLASH)
        type = "sketch";
      else // U_SPIFFS
        type = "filesystem";

      // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
      Serial.println("Start updating " + type); })
      .onEnd([]()
             { Serial.println("\nEnd"); })
      .onProgress([](unsigned int progress, unsigned int total)
                  { Serial.printf("Progress: %u%%\r", (progress / (total / 100))); })
      .onError([](ota_error_t error)
               {
      Serial.printf("Error[%u]: ", error);
      if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
      else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
      else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
      else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
      else if (error == OTA_END_ERROR) Serial.println("End Failed"); });


  ArduinoOTA.begin();
  SerialWIFI.begin();

  Serial.println("Ready");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
  Serial.print("Wifi Hostname: ");
  Serial.println(WiFi.getHostname());
  Serial.print("Arduino Hostname: ");
  Serial.print(ArduinoOTA.getHostname());Serial.println(".local");

  xTaskCreate(
      ota_handle,   /* Task function. */
//       "OTA_HANDLE", /* String with name of task. */
//       10000,        /* Stack size in bytes. */
//       NULL,         /* Parameter passed as input of the task */
//       1,            /* Priority of the task. */
//       NULL);        /* Task handle. */
// }
