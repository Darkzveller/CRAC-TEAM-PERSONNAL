/*#include <CAN.h>

void setup() {
  Serial.begin(9600);
  while (!Serial);

  Serial.println("CAN Sender");

  // start the CAN bus at 500 kbps
  if (!CAN.begin(1000E3)) {
    Serial.println("Starting CAN failed!");
    while (1);
  }
}

void loop() {
  // send packet: id is 11 bits, packet can contain up to 8 bytes of data
  Serial.print("Sending packet ... ");

  CAN.beginPacket(0x12);
  CAN.write('h');
  CAN.write('e');
  CAN.write('l');
  CAN.write('l');
  CAN.write('o');
  CAN.endPacket();

  Serial.println("done");

  delay(1000);

  // send extended packet: id is 29 bits, packet can contain up to 8 bytes of data
  Serial.print("Sending extended packet ... ");

  CAN.beginExtendedPacket(0xabcdef);
  CAN.write('w');
  CAN.write('o');
  CAN.write('r');
  CAN.write('l');
  CAN.write('d');
  CAN.endPacket();

  Serial.println("done");

  delay(1000);
}
*/
/*
#include <CAN.h>

void setup() {
  Serial.begin(9600);
  while (!Serial);

  Serial.println("CAN Receiver");

  // start the CAN bus at 500 kbps
  if (!CAN.begin(500E3)) {
    Serial.println("Starting CAN failed!");
    while (1);
  }
}

void loop() {
  // try to parse packet
  int packetSize = CAN.parsePacket();

  if (packetSize) {
    // received a packet
    Serial.print("Received ");

    if (CAN.packetExtended()) {
      Serial.print("extended ");
    }

    if (CAN.packetRtr()) {
      // Remote transmission request, packet contains no data
      Serial.print("RTR ");
    }

    Serial.print("packet with id 0x");
    Serial.print(CAN.packetId(), HEX);

    if (CAN.packetRtr()) {
      Serial.print(" and requested length ");
      Serial.println(CAN.packetDlc());
    } else {
      Serial.print(" and length ");
      Serial.println(packetSize);

      // only print packet data for non-RTR packets
      while (CAN.available()) {
        Serial.print((char)CAN.read());
      }
      Serial.println();
    }

    Serial.println();
  }
}
*/
/*
#include <Arduino.h>
#include <driver/twai.h> // Bibliothèque TWAI pour ESP32

// Configuration des pins
#define TWAI_TX_PIN GPIO_NUM_5 // Remplacez par le pin TX que vous utilisez
#define TWAI_RX_PIN GPIO_NUM_4 // Remplacez par le pin RX que vous utilisez

void setup() {
  Serial.begin(115200);

  // Configuration de TWAI
  twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(TWAI_TX_PIN, TWAI_RX_PIN, TWAI_MODE_NORMAL);
  twai_timing_config_t t_config = TWAI_TIMING_CONFIG_1MBITS(); // 1 Mbps
  twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL(); // Accepte tous les messages CAN

  // Initialisation de TWAI
  if (twai_driver_install(&g_config, &t_config, &f_config) == ESP_OK) {
    Serial.println("TWAI driver installé avec succès.");
  } else {
    Serial.println("Erreur lors de l'installation du driver TWAI.");
    return;
  }

  // Activation du TWAI
  if (twai_start() == ESP_OK) {
    Serial.println("TWAI démarré avec succès.");
  } else {
    Serial.println("Erreur lors du démarrage de TWAI.");
  }
  delay(5000);
}

void loop() {
  // Exemple : Envoi d'un message CAN
  twai_message_t message;
  message.identifier = 0x123; // ID CAN
  message.data_length_code = 8; // DLC : Nombre d'octets dans le message
  message.data[0] = 0xDE; // Exemple de données
  message.data[1] = 0xAD;
  message.data[2] = 0xBE;
  message.data[3] = 0xEF;
  message.data[4] = 0x00;
  message.data[5] = 0x01;
  message.data[6] = 0x02;
  message.data[7] = 0x03;

  if (twai_transmit(&message, pdMS_TO_TICKS(1000)) == ESP_OK) {
    Serial.println("Message envoyé avec succès.");
  } else {
    Serial.println("Erreur lors de l'envoi du message.");
  }

  // Exemple : Réception d'un message CAN
  twai_message_t rx_message;
  if (twai_receive(&rx_message, pdMS_TO_TICKS(1000)) == ESP_OK) {
    Serial.print("Message reçu : ID=");
    Serial.print(rx_message.identifier, HEX);
    Serial.print(" DLC=");
    Serial.print(rx_message.data_length_code);
    Serial.print(" Data=");
    for (int i = 0; i < rx_message.data_length_code; i++) {
      Serial.print(rx_message.data[i], HEX);
      Serial.print(" ");
    }
    Serial.println();
  } else {
    Serial.println("Pas de message reçu.");
  }

  delay(1000); // Pause d'une seconde
}*/

#include <Arduino.h>
#include "CAN_ESP32E.h"

void setup()
{
  Serial.begin(115200);
  setupCAN(1000E3);
}

void loop()
{
  // sendCANMessage();
  // delay(1000); // Pause d'une seconde

  readCANMessage();
}

