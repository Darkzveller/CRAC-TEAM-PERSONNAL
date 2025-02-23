#include "OTA.h" // Inclusion de la bibliothèque pour gérer l'OTA (Over-The-Air)
#include "ID_CAN.h"
#include "CAN_ESP32E.h"

// Je laisse le routeur choisir l'adresse IP et
// je fixe le nom de mon réseau Wi-Fi, ce qui simplifie les démarches, notamment lors des débogages
//  Informations de connexion WiFi
extern int x_low_byte, x_high_byte;
extern int y_low_byte, y_high_byte;
extern int t_low_byte, t_high_byte;
#define MON_TELEPHONE
// #define MA_FREEBOX
// #define MON_PC

const char *name_card_elec = "espmaitre";
// BESOIN DE ME SIMPLIFIER MA VIE
#ifdef MON_TELEPHONE                // Nom d'hôte de la carte ESP32
const char *ssid = "Me voici";      // SSID du réseau WiFi
const char *password = "youssef13"; // Mot de passe du réseau WiFi
#endif
#ifdef MA_FREEBOX                                                  // Nom d'hôte de la carte ESP32
const char *ssid = "Freebox-587F87";                               // SSID du réseau WiFi
const char *password = "subcrescat-degend@-parciore@2-adducturos"; // Mot de passe du réseau WiFi
#endif
#ifdef MON_PC                         // Nom d'hôte de la carte ESP32
const char *ssid = "Detective-Conan"; // SSID du réseau WiFi
const char *password = "99xS,304";    // Mot de passe du réseau WiFi
#endif
bool justepouraffichage = 0;
// Fonction pour gérer les opérations OTA dans une tâche séparée
void ota_handle(void *parameter)
{
  TickType_t xLastWakeTime;
  xLastWakeTime = xTaskGetTickCount();
  for (;;)
  {
    SerialWIFIActivites(); // Gestion des opérations OTA (vérifie si une mise à jour est en cours)
    ArduinoOTA.handle();
    vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(250));
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
    ESP.restart();
  }

  // Affiche le succès de la connexion
  Serial.println("");
  Serial.println("Connexion établie !");

  // Définition du port OTA (par défaut, il est à 3232)
  // ArduinoOTA.setPort(3232);

  // Définit le nom d'hôte pour les opérations OTA
  ArduinoOTA.setHostname(name_card_elec);

  // Pas de mot de passe par défaut pour l'OTA
  // ArduinoOTA.setPassword("onverra");

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
  TelnetStream.begin();

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

void receptionWIFI()
{
  if (TelnetStream.available())
  {
    String input = TelnetStream.readString(); // Lire la chaîne complète
    Serial.print("Reçu: ");
    Serial.println(input); // Afficher ce qui a été reçu
    int f = 0;
    // Supprimer les espaces ou caractères indésirables (comme le retour à la ligne)
    input.trim(); // Enlève les espaces superflus et les retours à la ligne

    if (input.equals("S")) // Vérifier si l'entrée est "stop"
    {
      // i = 0; // Remise à zéro de la variable
      TelnetStream.println("STOP BASE ROULANTE");
    }
    if (input.equals("M")) // Vérifier si l'entrée est "start"
    {
      TelnetStream.println("START TEST");
    }
  }
}

void affichage_commande_wifi()
{

  TelnetStream.println("S pour tout stopper");
  // TelnetStream.println("M pour tout mettre en marche");
}

void receptionWIFI(char ch)
{

  static int i = 0;
  static String chaine = "";
  String commande;
  String valeur;
  int index, length;
  int cmd = 0;

  if ((ch == 13) or (ch == 10))
  {
    index = chaine.indexOf(' ');
    length = chaine.length();
    if (index == -1)
    {
      commande = chaine;
      valeur = "";
    }
    else
    {
      commande = chaine.substring(0, index);
      valeur = chaine.substring(index + 1, length);
    }
    if (commande == "ROTATION")
    {
      int8_t sens = 0;
      cmd = valeur.toInt();
      if (cmd > 0)
      {
        if (cmd >= 1)
        {
          sens = 1;
        }
      }
      else if (cmd < 0)
      {
        if (cmd <= -1)
        {
          sens = -1;
        }
      }
      // cmd = fabs(cmd);
      cmd = cmd;
      uint8_t lowByte = cmd & 0xFF;         // Octet de poids faible
      uint8_t highByte = (cmd >> 8) & 0xFF; // Octet de poids fort

      TelnetStream.println();

      TelnetStream.printf("Send command Rotation with cons");
      TelnetStream.printf(" cmd %d", cmd);
      TelnetStream.printf(" sens %d", sens);
      TelnetStream.println();
      Serial.printf("Send command Rotation with cons");
      Serial.printf(" cmd %d", cmd);
      Serial.printf(" sens %d", sens);
      Serial.println();

      sendCANMessage(ROTATION, 0, 0, 4, highByte, lowByte, 0x7B, 0, 0, 0, 0);
    }
    if (commande == "LIGNE")
    {
      int8_t sens = 0;
      cmd = valeur.toInt();
      if (cmd > 0)
      {
        if (cmd >= 1)
        {
          sens = 1;
        }
      }
      else if (cmd < 0)
      {
        if (cmd <= -1)
        {
          sens = -1;
        }
      }
      // cmd = fabs(cmd);
      cmd = cmd;
      uint8_t lowByte = cmd & 0xFF;         // Octet de poids faible
      uint8_t highByte = (cmd >> 8) & 0xFF; // Octet de poids fort
      TelnetStream.println();
      TelnetStream.printf("Send command LIGNE with cons");
      TelnetStream.printf(" cmd %d", cmd);
      TelnetStream.printf(" lowByte %d", lowByte);
      TelnetStream.printf(" highByte %d", highByte);

      TelnetStream.println();
      Serial.printf("Send command LIGNE with cons");
      Serial.printf(" cmd %d", cmd);
      Serial.printf(" sens %d", sens);
      Serial.println();

      sendCANMessage(LIGNE_DROITE, 0, 0, 4, highByte, lowByte, 0x7B, 0, 0, 0, 0);
    }
    if (commande == "x")
    {
      cmd = valeur.toInt();

      uint8_t lowByte = cmd & 0xFF;         // Octet de poids faible
      uint8_t highByte = (cmd >> 8) & 0xFF; // Octet de poids fort
      x_low_byte = lowByte;
      x_high_byte = highByte;
      TelnetStream.println();

      TelnetStream.printf("Send command X with cons");
      TelnetStream.printf(" cmd %d", cmd);
      TelnetStream.println();
    }

    if (commande == "y")
    {
      cmd = valeur.toInt();

      uint8_t lowByte = cmd & 0xFF;         // Octet de poids faible
      uint8_t highByte = (cmd >> 8) & 0xFF; // Octet de poids fort
      y_low_byte = lowByte;
      y_high_byte = highByte;
      TelnetStream.println();

      TelnetStream.printf("Send command y with cons");
      TelnetStream.printf(" cmd %d", cmd);
      TelnetStream.println();
    }
    if (commande == "t")
    {
      cmd = valeur.toInt();

      uint8_t lowByte = cmd & 0xFF;         // Octet de poids faible
      uint8_t highByte = (cmd >> 8) & 0xFF; // Octet de poids fort
      t_low_byte = lowByte;
      t_high_byte = highByte;
      TelnetStream.println();

      TelnetStream.printf("Send command t with cons");
      TelnetStream.printf(" cmd %d", cmd);
      TelnetStream.println();
      sendCANMessage(XYTHETA, 0, 0, 4, x_high_byte, x_low_byte, y_high_byte, y_low_byte, t_high_byte, t_low_byte, 0);
    }
    if (commande == "RESTART")
    {
      TelnetStream.println();

      TelnetStream.printf("Send command RESTART");
      TelnetStream.println();
      Serial.printf("Send command RESTART ");
      Serial.printf("Send command OFF_1 Bat_1 ");
      Serial.printf("Send command OFF_2 Bat_2 ");Serial.printf("Send command OFF_3 Bat_3 ");
      Serial.println();

      sendCANMessage(ESP32_RESTART, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
      sendCANMessage(INTERRUPTEUR_BATT1, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0);
      sendCANMessage(INTERRUPTEUR_BATT2, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0);
      sendCANMessage(INTERRUPTEUR_BATT3, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0);
    }

    if (commande == "ON1")
    {
      TelnetStream.println();

      TelnetStream.printf("Send command ON_1 Bat_1 ");
      TelnetStream.println();
      Serial.printf("Send command ON_1 Bat_1");
      Serial.println();

      sendCANMessage(INTERRUPTEUR_BATT1, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0);
    }
    if (commande == "OFF1")
    {
      TelnetStream.println();

      TelnetStream.printf("Send command OFF_1 Bat_1 ");
      TelnetStream.println();
      Serial.printf("Send command OFF_1 Bat_1");
      Serial.println();

      sendCANMessage(INTERRUPTEUR_BATT1, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0);
    }
    if (commande == "ON2")
    {
      TelnetStream.println();

      TelnetStream.printf("Send command ON_2 Bat_2 ");
      TelnetStream.println();
      Serial.printf("Send command ON_2 Bat_2");
      Serial.println();

      sendCANMessage(INTERRUPTEUR_BATT2, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0);
    }
    if (commande == "OFF2")
    {
      TelnetStream.println();

      TelnetStream.printf("Send command OFF_2 Bat_2 ");
      TelnetStream.println();
      Serial.printf("Send command OFF_2 Bat_2");
      Serial.println();

      sendCANMessage(INTERRUPTEUR_BATT2, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0);
    }
    if (commande == "ON3")
    {
      TelnetStream.println();

      TelnetStream.printf("Send command ON_3 Bat_3 ");
      TelnetStream.println();
      Serial.printf("Send command ON_3 Bat_3");
      Serial.println();

      sendCANMessage(INTERRUPTEUR_BATT3, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0);
    }
    if (commande == "OFF3")
    {
      TelnetStream.println();

      TelnetStream.printf("Send command OFF_3 Bat_3 ");
      TelnetStream.println();
      Serial.printf("Send command OFF_3 Bat_3");
      Serial.println();

      sendCANMessage(INTERRUPTEUR_BATT3, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0);
    }

    
    chaine = "";
  }
  else
  {
    chaine += ch;
  }
}
void SerialWIFIActivites()
{
  while (TelnetStream.available() > 0) // tant qu'il y a des caractères à lire
  {
    if (justepouraffichage == 0)
    {
      TelnetStream.println("Bien veneu dans le terminal WIFI");
      Serial.println("Bien veneu dans le terminal WIFI");
    }
    justepouraffichage = 1;

    receptionWIFI(TelnetStream.read());
  }
}