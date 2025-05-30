
#include <Arduino.h>
#include "ID_CAN.h"
#include "OTA.h" // Inclusion de la bibliothèque pour gérer l'OTA (Over-The-Air)
#include "CAN_ESP32E.h"
#include "STRATEGIE.h"
#include "Variable.h"
static bool start_match = 0;
int etat_match = 0;

int data[10] = {0};
int id = 0;
int vitesse = 90;
#define PIN_CHOIX_COULEUR 32
#define PIN_5V_FDC 25
#define PIN_READ_FDC 33
#define TIME_TO_SEND_DATA_CAN 20
// #define HOMOLOGATION
// #define UN_SPOT
void setup()
{
  Serial.begin(115200);
  Serial.printf("CACA");
  setupCAN(1000E3);
  // setupOTA();
  // Pour le finde course
  pinMode(PIN_5V_FDC, OUTPUT);
  pinMode(PIN_READ_FDC, INPUT_PULLDOWN);
  digitalWrite(PIN_5V_FDC, true);

  // Pour choisir couleur de l'équipe
  pinMode(PIN_CHOIX_COULEUR, INPUT_PULLDOWN);
  affichage_commande_wifi();
}
void loop()
{
  readCANMessage();
  start_match = digitalRead(PIN_READ_FDC);
  static bool choix_couleur_equipe = digitalRead(32);
  // Serial.printf("Val couleur %d \n", choix_couleur_equipe);
#ifdef HOMOLOGATION
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
    Serial.printf("Esp restart ");
    sendCANMessage(ESP32_RESTART, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
    // Metttre les 5 secondes pour laisser le temps au robot de s'immobiliser
    delay(5000);
    if (choix_couleur_equipe == 0)
    {
      Serial.printf("Strat jaune + recalage ");

      strategie_jaune_homologation();
    }
    else
    {
      Serial.printf("Strat bleu + recalage ");
      strategie_bleu_homologation();
    }
    Serial.printf("Start match ");

    sendCANMessage(START_ROBOT_MATCH, 0, 0, 8, 0, 0, 0, 0, 0, 0, 0, 0);
    delay(100);
    Serial.printf("ON2 ");

    sendCANMessage(INTERRUPTEUR_BATT2, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0);
    delay(100);
    Serial.printf("Message envoyé et début de match \n");
    etat_match = 2;
    break;

  case 2:
    break;

  default:
    break;
  }

#endif

#ifdef UN_SPOT
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
    Serial.printf("Esp restart ");
    sendCANMessage(ESP32_RESTART, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
    // Metttre les 5 secondes pour laisser le temps au robot de s'immobiliser
    delay(5000);
    if (choix_couleur_equipe == 0)
    {
      Serial.printf("recalage jaune");
      send_recalage(1225, 250, 90);
    }
    else
    {
      Serial.printf("recalage bleu");
      send_recalage(1775, 250, 90);
    }
    Serial.printf("Start match ");

    sendCANMessage(START_ROBOT_MATCH, 0, 0, 8, 0, 0, 0, 0, 0, 0, 0, 0);
    delay(TIME_TO_SEND_DATA_CAN);
    Serial.printf("ON1 ");
    sendCANMessage(INTERRUPTEUR_BATT1, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0);
    delay(TIME_TO_SEND_DATA_CAN);
    Serial.printf("ON2 ");
    sendCANMessage(INTERRUPTEUR_BATT2, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0);
    delay(TIME_TO_SEND_DATA_CAN);
    Serial.printf("ON3 ");
    sendCANMessage(INTERRUPTEUR_BATT3, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0);
    delay(TIME_TO_SEND_DATA_CAN);

    sendCANMessage(CONSTRUIRE_AVANT_PREPARER, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0);
    delay(TIME_TO_SEND_DATA_CAN);

    etat_match = 2;
    break;
  case 2:
    send_ligne_droite(150);
    etat_match = 3;
    break;
  case 3:
    if (rxMsg.id == ACKNOWLEDGE_BASE_ROULANTE)
    {
      rxMsg.id = 0;

      delay(1000);

      send_rotation(-45);

      etat_match = 4;
    }
    break;
  case 4:
    if (rxMsg.id == ACKNOWLEDGE_BASE_ROULANTE)
    {
      rxMsg.id = 0;
      delay(1000);
      send_ligne_droite(175);
      etat_match = 5;
    }
    break;
  case 5:
    if (rxMsg.id == ACKNOWLEDGE_BASE_ROULANTE)
    {
      rxMsg.id = 0;

      delay(1000);
      send_rotation(40);
      etat_match = 6;
    }

  case 6:
    if (rxMsg.id == ACKNOWLEDGE_BASE_ROULANTE)
    {
      rxMsg.id = 0;

      delay(1000);
      send_ligne_droite(600);
      etat_match = 7;
    }

    break;

  case 7:
    if (rxMsg.id == ACKNOWLEDGE_BASE_ROULANTE)
    {
      rxMsg.id = 0;

      send_x_y_theta(600, 1500, 0);
      delay(200);
      sendCANMessage(CONSTRUIRE_AVANT_2ETAGE, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0);

      etat_match = 8;
    }

    break;
  case 8:
    break;

  default:
    break;
  }

#endif
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
