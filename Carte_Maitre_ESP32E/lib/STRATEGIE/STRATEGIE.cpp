#include "ID_CAN.h"
#include "STRATEGIE.h"
#include "CAN_ESP32E.h"

uint8_t nbr_strat = 0;

void strategie_jaune_homologation()
{
  send_recalage(1225, 139, 90);
  delay(200);
  nbr_strat = 1;
  // uint16_t x_homologation[nbr_ps] = {1225, 750, 750, 300};
  // uint16_t y_homologation[nbr_ps] = {750, 750, 1500, 1800};

  uint16_t x_homologation[nbr_strat] = {600};
  uint16_t y_homologation[nbr_strat] = {1500};
  // uint16_t t_homologation[nbr_strat] = {-45};

  for (int i = 0; i < nbr_strat; i++)
  {
    Serial.println("message");
    uint16_t cmd_x = x_homologation[i];
    uint16_t cmd_y = y_homologation[i];

    uint8_t lowByte_x = cmd_x & 0xFF;         // Octet de poids faible
    uint8_t highByte_x = (cmd_x >> 8) & 0xFF; // Octet de poids fort

    uint8_t lowByte_y = cmd_y & 0xFF;         // Octet de poids faible
    uint8_t highByte_y = (cmd_y >> 8) & 0xFF; // Octet de poids fort

    sendCANMessage(POLAIRE, 0, 0, 8, i, highByte_x, lowByte_x, highByte_y, lowByte_y, nbr_strat, 0, 0);
    delay(10);
  }
}

void strategie_bleu_homologation()
{
  send_recalage(1775, 139, 90);
  delay(200);

  nbr_strat = 1;

  uint16_t x_homologation[nbr_strat] = {3000 - 600};
  uint16_t y_homologation[nbr_strat] = {3000 - 1500};

  for (int i = 0; i < nbr_strat; i++)
  {
    Serial.println("message");
    uint16_t cmd_x = x_homologation[i];
    uint16_t cmd_y = y_homologation[i];

    uint8_t lowByte_x = cmd_x & 0xFF;         // Octet de poids faible
    uint8_t highByte_x = (cmd_x >> 8) & 0xFF; // Octet de poids fort

    uint8_t lowByte_y = cmd_y & 0xFF;         // Octet de poids faible
    uint8_t highByte_y = (cmd_y >> 8) & 0xFF; // Octet de poids fort

    sendCANMessage(POLAIRE, 0, 0, 8, i, highByte_x, lowByte_x, highByte_y, lowByte_y, nbr_strat, 0, 0);
    delay(20);
  }
}

void send_recalage(int x, int y, int theta)
{
  uint16_t cmd_x = x;
  uint16_t cmd_y = y;
  uint16_t cmd_theta = theta;

  uint8_t lowByte_x = cmd_x & 0xFF;         // Octet de poids faible
  uint8_t highByte_x = (cmd_x >> 8) & 0xFF; // Octet de poids fort

  uint8_t lowByte_y = cmd_y & 0xFF;                 // Octet de poids faible
  uint8_t highByte_y = (cmd_y >> 8) & 0xFF;         // Octet de poids fort
  uint8_t lowByte_theta = cmd_theta & 0xFF;         // Octet de poids faible
  uint8_t highByte_theta = (cmd_theta >> 8) & 0xFF; // Octet de poids fort

  sendCANMessage(RECALAGE, 0, 0, 8, 0, 1, highByte_x, lowByte_x, 0, 0, 0, 0);
  delay(40);
  sendCANMessage(RECALAGE, 0, 0, 8, 0, 2, highByte_y, lowByte_y, 0, 0, 0, 0);
  delay(40);
  sendCANMessage(RECALAGE, 0, 0, 8, 0, 3, highByte_theta, lowByte_theta, 0, 0, 0, 0);
  delay(20);
}

void send_x_y_theta(int x, int y, int theta)
{

  nbr_strat = 1;
  for (int i = 0; i < nbr_strat; i++)
  {
    Serial.println("message");
    uint16_t cmd_x = x;
    uint16_t cmd_y = y;
    uint16_t cmd_theta = theta;

    uint8_t lowByte_x = cmd_x & 0xFF;         // Octet de poids faible
    uint8_t highByte_x = (cmd_x >> 8) & 0xFF; // Octet de poids fort

    uint8_t lowByte_y = cmd_y & 0xFF;                 // Octet de poids faible
    uint8_t highByte_y = (cmd_y >> 8) & 0xFF;         // Octet de poids fort
    uint8_t lowByte_theta = cmd_theta & 0xFF;         // Octet de poids faible
    uint8_t highByte_theta = (cmd_theta >> 8) & 0xFF; // Octet de poids fort

    sendCANMessage(POLAIRE, 0, 0, 8, i, highByte_x, lowByte_x, highByte_y, lowByte_y, nbr_strat, highByte_theta, lowByte_theta);
    delay(20);
  }
}

void send_ligne_droite(int cmd)
{

  Serial.println("LIGNE");

  cmd = cmd;
  uint8_t lowByte = cmd & 0xFF;         // Octet de poids faible
  uint8_t highByte = (cmd >> 8) & 0xFF; // Octet de poids fort

  sendCANMessage(LIGNE_DROITE, 0, 0, 8, highByte, lowByte, 0x7B, 0, 0, 0, 0, 0);
}

void send_rotation(int cmd)
{

  Serial.println("Rotation ");

  cmd = cmd;
  uint8_t lowByte = cmd & 0xFF;         // Octet de poids faible
  uint8_t highByte = (cmd >> 8) & 0xFF; // Octet de poids fort

  sendCANMessage(ROTATION, 0, 0, 8, highByte, lowByte, 0x7B, 0, 0, 0, 0, 0);
}
