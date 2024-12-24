
#include <Arduino.h>
#include "ID_CAN.h"
#include "OTA.h" // Inclusion de la bibliothèque pour gérer l'OTA (Over-The-Air)
#include "CAN_ESP32E.h"

int etat = 0;

int data[10] = {0};
int id = 0;
void setup()
{
  Serial.begin(115200);
  Serial.printf("CACA");
  setupCAN(1000E3);
  setupOTA();
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
      cmd = fabs(cmd);
      uint8_t lowByte = cmd & 0xFF;         // Octet de poids faible
      uint8_t highByte = (cmd >> 8) & 0xFF; // Octet de poids fort

      Serial.printf("Send command Rotation with cons");
      Serial.printf(" cmd %d", cmd);
      Serial.printf(" sens %d", sens);
      Serial.println();
      sendCANMessage(ROTATION, 0, 0, 4, highByte, lowByte, sens, 100, 0, 0, 0);
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
      cmd = fabs(cmd);
      uint8_t lowByte = cmd & 0xFF;         // Octet de poids faible
      uint8_t highByte = (cmd >> 8) & 0xFF; // Octet de poids fort

      Serial.printf("Send command Ligne with cons");
      Serial.printf(" cmd %d", cmd);
      Serial.printf(" sens %d", sens);
      Serial.println();
      sendCANMessage(LIGNE_DROITE, 0, 0, 4, highByte, lowByte, sens, 100, 0, 0, 0);
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
    reception(Serial.read());
  }
}
