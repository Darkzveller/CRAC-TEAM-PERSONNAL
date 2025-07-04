#include <Arduino.h>
#include <driver/twai.h> // Bibliothèque TWAI pour ESP32
#include "OTA.h"
#include "ID_CAN.h"
#include "CAN_ESP32E.h"
#include "Variable.h"
#include "USE_FUNCTION.h"

extern int data[10];
extern int id;
// Configuration des pins
#define TWAI_TX_PIN GPIO_NUM_19 // Remplacez par le pin TX que vous utilisez
#define TWAI_RX_PIN GPIO_NUM_18 // Remplacez par le pin RX que vous utilisez
#define Tcan 5
#define TYPE_DEPLACEMENT_IMMOBILE 1
#define TYPE_DEPLACEMENT_LIGNE_DROITE 2
#define TYPE_DEPLACEMENT_ROTATION 3
#define TYPE_DEPLACEMENT_X_Y_THETA 4
#define TYPE_DEPLACEMENT_RECALAGE 5
#define TYPE_VIDE 6

void setupCAN(int baudrate)
{
    // Configuration de TWAI
    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(TWAI_TX_PIN, TWAI_RX_PIN, TWAI_MODE_NORMAL);
    twai_timing_config_t t_config; // Déclaration unique en dehors des blocs conditionnels

    if (baudrate == 1000E3)
    {
        t_config = TWAI_TIMING_CONFIG_1MBITS(); // 1 Mbps
    }
    else if (baudrate == 500E3)
    {
        t_config = TWAI_TIMING_CONFIG_500KBITS(); // 500 kbps
    }
    else if (baudrate == 250E3)
    {
        t_config = TWAI_TIMING_CONFIG_250KBITS(); // 250 kbps
    }
    else if (baudrate == 125E3)
    {
        t_config = TWAI_TIMING_CONFIG_125KBITS(); // 125 kbps
    }
    else if (baudrate == 100E3)
    {
        t_config = TWAI_TIMING_CONFIG_100KBITS(); // 100 kbps
    }
    else if (baudrate == 50E3)
    {
        t_config = TWAI_TIMING_CONFIG_50KBITS(); // 50 kbps
    }
    else if (baudrate == 25E3)
    {
        t_config = TWAI_TIMING_CONFIG_25KBITS(); // 25 kbps (approximation pour 20 kbps)
    }
    else
    {
        Serial.println("Erreur : Baudrate non supporté.");
        return;
    }

    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL(); // Accepte tous les messages CAN

    // Initialisation du driver TWAI avec les configurations définies
    if (twai_driver_install(&g_config, &t_config, &f_config) == ESP_OK)
    {
        Serial.println("TWAI driver installé avec succès.");
    }
    else
    {
        Serial.println("Erreur lors de l'installation du driver TWAI.");
        return;
    }

    // Démarrage du driver TWAI
    if (twai_start() == ESP_OK)
    {
        Serial.println("TWAI démarré avec succès.");
    }
    else
    {
        Serial.println("Erreur lors du démarrage de TWAI.");
    }
}
void sendCANMessage(int id, int ext, int rtr, int length, int data0, int data1, int data2, int data3, int data4, int data5, int data6, int data7)
{
    // Exemple : Envoi d'un message CAN
    twai_message_t message;
    message.identifier = id; // ID CAN
    message.extd = ext;
    message.rtr = rtr;                 // Active le mode identifiant étendu (29 bits)
    message.data_length_code = length; // DLC : Nombre d'octets dans le message
    message.data[0] = data0;           // Données a envoyés
    message.data[1] = data1;
    message.data[2] = data2;
    message.data[3] = data3;
    message.data[4] = data4;
    message.data[5] = data5;
    message.data[6] = data6;
    message.data[7] = data7;
    // Envoi du message avec un délai d'attente de 1000 ms
    if (twai_transmit(&message, pdMS_TO_TICKS(10)) == ESP_OK)
    {
        Serial.println("Message envoyé avec succès.");
    }
    else
    {
        Serial.println("Erreur lors de l'envoi du message.");
    }
}

// void readCANMessage()
// {
//     // Exemple : Réception d'un message CAN
//     twai_message_t rx_message;

//     if (twai_receive(&rx_message, pdMS_TO_TICKS(Tcan)) == ESP_OK)
//     { /*
//          Serial.print("Message reçu : ID=");
//          Serial.print(rx_message.identifier, HEX);
//          Serial.print(" DLC=");
//          Serial.print(rx_message.data_length_code);
//          Serial.print(" Data=");
//          for (int i = 0; i < rx_message.data_length_code; i++)
//          {
//              Serial.print(rx_message.data[i], HEX);
//              Serial.print(" ");
//          }
//          Serial.println();
//     */

//         // Si l'ID est valide, traitement du message
//         Serial.print(" Message reçu : ID=");
//         Serial.print(rx_message.identifier, HEX); // Affiche l'ID du message en hexadécimal
//         // Serial.print(" EXT=");
//         // Serial.print(rx_message.extd); // Affiche le Data Length Code (DLC)
//         // Serial.print(" RTR=");
//         // Serial.print(rx_message.rtr); // Affiche le Data Length Code (DLC)

//         // Serial.print(" DLC=");
//         // Serial.print(rx_message.data_length_code); // Affiche le Data Length Code (DLC)
//         Serial.print(" Data=");

//         id = rx_message.identifier;
//         // rxMsg.extd = rx_message.extd;
//         // rxMsg.rtr = rx_message.rtr;
//         // rxMsg.lenght = rx_message.data_length_code;
//         // Affichage des données du message
//         for (int i = 0; i < rx_message.data_length_code; i++)
//         {
//             data[i] = rx_message.data[i];
//             Serial.print(data[i], HEX); // Affiche chaque octet de données en hexadécimal
//             Serial.print(" ");
//         }
//      Serial.println();

//        /* {

//              else
//              {
//                  // Si l'ID n'est pas pris en compte, on l'ignore
//                  Serial.print(" Message reçu avec ID non pris en compte : ");
//                  Serial.print(rx_message.identifier, HEX);
//              }
//         } /*
//          else
//          {
//              Serial.print(" Pas de message reçu.");
//          }    Serial.println();
//      */
//     }
// }

void readCANMessage()
{
    // Exemple : Réception d'un message CAN
    twai_message_t rx_message;

    if (twai_receive(&rx_message, pdMS_TO_TICKS(Tcan)) == ESP_OK)
    { /*
         Serial.print("Message reçu : ID=");
         Serial.print(rx_message.identifier, HEX);
         Serial.print(" DLC=");
         Serial.print(rx_message.data_length_code);
         Serial.print(" Data=");
         for (int i = 0; i < rx_message.data_length_code; i++)
         {
             Serial.print(rx_message.data[i], HEX);
             Serial.print(" ");
         }
         Serial.println();
    */
        if (messageCANForMe(rx_message.identifier))
        {

            // Si l'ID est valide, traitement du message
            // Serial.print(" Message reçu : ID=");
            // Serial.print(rx_message.identifier, HEX); // Affiche l'ID du message en hexadécimal
            // Serial.print(" EXT=");
            // Serial.print(rx_message.extd); // Affiche le Data Length Code (DLC)
            // Serial.print(" RTR=");
            // Serial.print(rx_message.rtr); // Affiche le Data Length Code (DLC)

            // Serial.print(" DLC=");
            // Serial.print(rx_message.data_length_code); // Affiche le Data Length Code (DLC)
            // Serial.print(" Data=");

            rxMsg.id = rx_message.identifier;
            rxMsg.extd = rx_message.extd;
            rxMsg.rtr = rx_message.rtr;
            rxMsg.lenght = rx_message.data_length_code;
            // Affichage des données du message
            for (int i = 0; i < rx_message.data_length_code; i++)
            {
                rxMsg.data[i] = rx_message.data[i];
                // Serial.print(rxMsg.data[i], HEX); // Affiche chaque octet de données en hexadécimal
                // Serial.print(" ");
                // TelnetStream.print(rxMsg.data[i], HEX); // Affiche chaque octet de données en hexadécimal
                // TelnetStream.print(" ");
            }
        } /*
     else
     {
         // Si l'ID n'est pas pris en compte, on l'ignore
         Serial.print(" Message reçu avec ID non pris en compte : ");
         Serial.print(rx_message.identifier, HEX);
     }*/
    } /*
     else
     {
         Serial.print(" Pas de message reçu.");
     }    Serial.println();
 */
}

bool messageCANForMe(uint16_t ID)
{
    switch (ID)
    {
    case CARTE_MAITRE:
        return true;
        break;

    case ACKNOWLEDGE_BASE_ROULANTE:
        return true;
        break;
    case ODO_SEND:

        // Serial.printf("X = %d ", fusion_octet(rxMsg.data[0], rxMsg.data[1]));
        // Serial.printf("Y = %d ", fusion_octet(rxMsg.data[2], rxMsg.data[3]));
        Serial.printf("Theta = %d ", fusion_octet(rxMsg.data[4], rxMsg.data[5]));
        Serial.println();
        return true;
        break;

    default:
        return false;
        break;
    }
    return false;
}
