
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

uint16_t cmd_x = 1225;
uint16_t cmd_y = 139;
uint16_t cmd_theta = 90;

uint8_t lowByte_x = cmd_x & 0xFF;         // Octet de poids faible
uint8_t highByte_x = (cmd_x >> 8) & 0xFF; // Octet de poids fort

uint8_t lowByte_y = cmd_y & 0xFF;                 // Octet de poids faible
uint8_t highByte_y = (cmd_y >> 8) & 0xFF;         // Octet de poids fort
uint8_t lowByte_theta = cmd_theta & 0xFF;         // Octet de poids faible
uint8_t highByte_theta = (cmd_theta >> 8) & 0xFF; // Octet de poids fort

void setup()
{
  Serial.begin(115200);
  Serial.printf("CACA");
  setupCAN(1000E3);
  // setupOTA();
  int i = 0;
  // Pour le finde course
  pinMode(25, OUTPUT);
  pinMode(33, INPUT_PULLDOWN);
  digitalWrite(25, true);

  // Pour choisir couleur de l'équipe
  pinMode(32, INPUT_PULLDOWN);

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
int etat_match = 0;
void loop()
{
  readCANMessage();
  static bool start_match = 0;
  start_match = digitalRead(33);
Serial.printf("Val couleur %d ",digitalRead(32));
  switch (etat_match)
  {
  case 0:
    if (start_match)
    {
      Serial.printf("Start match ");
      Serial.println(start_match);

      etat_match = 1;
    }
    break;
  case 1:
    sendCANMessage(ESP32_RESTART, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);

    delay(5000);

    sendCANMessage(RECALAGE, 0, 0, 8, 0, 1, highByte_x, lowByte_x, 0, 0, 0, 0);
    delay(40);
    sendCANMessage(RECALAGE, 0, 0, 8, 0, 2, highByte_y, lowByte_y, 0, 0, 0, 0);
    delay(40);
    sendCANMessage(RECALAGE, 0, 0, 8, 0, 3, highByte_theta, lowByte_theta, 0, 0, 0, 0);
    delay(20);
    sendCANMessage(START_ROBOT_MATCH, 0, 0, 8, 0, 0, 0, 0, 0, 0, 0, 0);
    delay(100);
    sendCANMessage(INTERRUPTEUR_BATT2, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0);
    delay(100);
    strategie_jaune_homologation();

    etat_match = 2;
    break;

  case 2:
    Serial.printf("Message envoyé et début de match \n");
    break;

  default:
    break;
  }

  // void sendCANMessage(int id, int ext, int rtr, int length, int data0, int data1, int data2, int data3, int data4, int data5, int data6, int data7)

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
