
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

void serialEvent()
{
  while (Serial.available() > 0) // tant qu'il y a des caractères à lire
  {
    // reception(Serial.read());
    char caractere = Serial.read();
    receptionWIFI(caractere);
    Serial.print(caractere);
  }
}
