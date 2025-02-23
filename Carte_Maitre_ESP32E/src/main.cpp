
#include <Arduino.h>
#include "ID_CAN.h"
#include "OTA.h" // Inclusion de la bibliothèque pour gérer l'OTA (Over-The-Air)
#include "CAN_ESP32E.h"

int etat = 0;

int data[10] = {0};
int id = 0;
int vitesse = 90;
int x_low_byte, x_high_byte;
int y_low_byte, y_high_byte;
int t_low_byte, t_high_byte;

void setup()
{
  Serial.begin(115200);
  Serial.printf("CACA");
  setupCAN(1000E3);
  // setupOTA();
  int i = 0;
  // while (!TelnetStream.available())
  // {
  //   delay(500);                                             // Attente de 500 ms avant de vérifier à nouveau
  //   Serial.println("Aucun client connecté, en attente..."); // Message indiquant qu'il n'y a pas de client connecté
  //   i++;
  //   // if (i == 22)
  //   // {
  //   //   esp_restart();
  //   // }
  // }
  affichage_commande_wifi();
}

void loop()
{
  readCANMessage();
  // Serial.printf("Send command ON ALL Bat ");
  // Serial.printf("Send command Rotation with cons");
  // Serial.println();

      // sendCANMessage(ROTATION, 0, 0, 4, 22, 22, 0x7B, 0, 0, 0, 0);

  // sendCANMessage(INTERRUPTEUR_BATT1, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0);
  // sendCANMessage(INTERRUPTEUR_BATT2, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0);
  // sendCANMessage(INTERRUPTEUR_BATT3, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0);

  // Serial.println();
  // delay(150);
  // sendCANMessage(ROTATION, 0, 0, 4, 0, 0x5C, 0x1, 0x7B, 0, 0, 0);
}

void reception(char ch)
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

      Serial.printf("Send command Rotation with cons");
      Serial.printf(" cmd %d", cmd);
      Serial.printf(" sens %d", sens);
      Serial.println();
      sendCANMessage(ROTATION, 0, 0, 4, highByte, lowByte, vitesse, 0, 0, 0, 0);
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

      Serial.printf("Send command Ligne with cons");
      Serial.printf(" cmd %d", cmd);
      Serial.printf(" sens %d", sens);
      Serial.println();
      sendCANMessage(LIGNE_DROITE, 0, 0, 4, highByte, lowByte, vitesse, 0, 0, 0, 0);
    }

    if (commande == "x")
    {
      cmd = valeur.toInt();

      uint8_t lowByte = cmd & 0xFF;         // Octet de poids faible
      uint8_t highByte = (cmd >> 8) & 0xFF; // Octet de poids fort
      x_low_byte = lowByte;
      x_high_byte = highByte;
      Serial.printf("Send command X with cons");
      Serial.printf(" cmd %d", cmd);
      Serial.println();
    }

    if (commande == "y")
    {
      cmd = valeur.toInt();

      uint8_t lowByte = cmd & 0xFF;         // Octet de poids faible
      uint8_t highByte = (cmd >> 8) & 0xFF; // Octet de poids fort
      y_low_byte = lowByte;
      y_high_byte = highByte;

      Serial.printf("Send command y with cons");
      Serial.printf(" cmd %d", cmd);
      Serial.println();
    }
    if (commande == "t")
    {
      cmd = valeur.toInt();

      uint8_t lowByte = cmd & 0xFF;         // Octet de poids faible
      uint8_t highByte = (cmd >> 8) & 0xFF; // Octet de poids fort
      t_low_byte = lowByte;
      t_high_byte = highByte;

      Serial.printf("Send command t with cons");
      Serial.printf(" cmd %d", cmd);
      Serial.println();
      sendCANMessage(XYTHETA, 0, 0, 4, x_high_byte, x_low_byte, y_high_byte, y_low_byte, t_high_byte, t_low_byte, 0);
    }
    if (commande == "RESTART")
    {
      Serial.printf("Send command RESTART");
      Serial.println();
      sendCANMessage(ESP32_RESTART, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
    }

    chaine = "";
  }
  else
  {
    chaine += ch;
  }
}

void serialEvent()
{
  while (Serial.available() > 0) // tant qu'il y a des caractères à lire
  {
    // reception(Serial.read());
    receptionWIFI(Serial.read());
  }
}
