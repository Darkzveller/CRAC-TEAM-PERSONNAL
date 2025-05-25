#include <mat.h>
#include <Arduino.h>
#include "OTA.h" // Inclusion de la bibliothèque OTA (Over-The-Air) pour les mises à jour sans fil
#include "Variable.h"
#include "MOTEUR.h"
#include "EncoderManager.h"
#include "ASSERVISSEMENT.h"
#include "MOUVEMENT.h"
#include "ID_CAN.h"
#include "CAN_ESP32E.h"
#include "USE_FUNCTION.h"

float rectificateur_coeeff = 1;

void controle(void *parameters)
{
    TickType_t xLastWakeTime;
    xLastWakeTime = xTaskGetTickCount();
    while (1)
    {
        read_x_y_theta();
        static int pp = 1;

        if (pp == 0)
        {
            liste.nbr_passage = 0;
            liste.x_polaire[0] = 0;
            liste.y_polaire[0] = 0;
            // liste.x_polaire[0] = 250;
            // liste.y_polaire[0] = 500;

            // liste.x_polaire[1]=500;
            // liste.y_polaire[1] = 250;

            // liste.nbr_passage = 0;
            // liste.x_polaire[0] = 0;
            // liste.y_polaire[0] = 0;
            // // liste.x_polaire[1]=500;
            // liste.y_polaire[1] = 250;
            liste.rotation_polaire[0] = 90;
            liste.checksum_nbr_passage = liste.nbr_passage;

            liste.general_purpose = TYPE_DEPLACEMENT_X_Y_POLAIRE;
            flag_fin_mvt = false;
            rxMsg.id = 0;
            liste.compteur_point_de_passage_polaire = 0;

            pp = 1;
        }

        switch (liste.general_purpose)
        {
        case TYPE_DEPLACEMENT_LIGNE_DROITE:
            liste.vitesse_croisiere = SPEED_NORMAL;
            Serial.printf("TYPE_DEPLACEMENT_LIGNE_DROITE ");
            // TelnetStream.printf("TYPE_DEPLACEMENT_LIGNE_DROITE ");

            ligne_droite(liste.distance, liste.vitesse_croisiere);
            if (return_flag_asser_roue())
            {
                sendCANMessage(ACKNOWLEDGE_BASE_ROULANTE, 0, 0, 8, true, 0, 0, 0, 0, 0, 0, 0);
                liste.general_purpose = TYPE_DEPLACEMENT_IMMOBILE;
            }
            // TelnetStream.println();

            break;
        case TYPE_DEPLACEMENT_ROTATION:

            Serial.printf("TYPE_DEPLACEMENT_ROTATION ");
            // TelnetStream.printf("TYPE_DEPLACEMENT_ROTATION ");

            liste.vitesse_croisiere = SPEED_NORMAL;
            rotation(liste.angle, liste.vitesse_croisiere);
            // TelnetStream.println();

            if (return_flag_asser_roue())
            {
                consigne_theta_prec = degrees(theta_robot);
                sendCANMessage(ACKNOWLEDGE_BASE_ROULANTE, 0, 0, 8, true, 0, 0, 0, 0, 0, 0, 0);
                liste.general_purpose = TYPE_DEPLACEMENT_IMMOBILE;
            }
            break;
        case TYPE_DEPLACEMENT_IMMOBILE:
            consigne_position_droite = consigne_odo_droite_prec;
            consigne_position_gauche = consigne_odo_gauche_prec;
            Serial.printf(" TYPE_DEPLACEMENT_IMMOBILE");
            // TelnetStream.printf(" TYPE_DEPLACEMENT_IMMOBILE");

            liste.general_purpose = TYPE_VIDE;
            sendCANMessage(ACKNOWLEDGE_BASE_ROULANTE, 0, 0, 8, true, 0, 0, 0, 0, 0, 0, 0);
            // TelnetStream.println();

            break;
        case TYPE_DEPLACEMENT_X_Y_POLAIRE:
            Serial.printf(" TYPE_DEPLACEMENT_X_Y_POLAIRE ");
            // TelnetStream.printf(" TYPE_DEPLACEMENT_IMMOBILE");
            // Serial.printf(" liste.x_polaire %f ", liste.x_polaire[liste.compteur_point_de_passage_polaire]);
            // Serial.printf(" liste.y_polaire %f ", liste.y_polaire[liste.compteur_point_de_passage_polaire]);
            Serial.printf(" liste.cptps %d ", liste.compteur_point_de_passage_polaire);
            TelnetStream.printf(" liste.cptps %d ", liste.compteur_point_de_passage_polaire);

            asser_polaire_tick(liste.x_polaire[liste.compteur_point_de_passage_polaire], liste.y_polaire[liste.compteur_point_de_passage_polaire], liste.rotation_polaire[liste.compteur_point_de_passage_polaire], liste.deceleration_polaire);
            // TelnetStream.println();

            if (flag_fin_mvt)
            {

                sendCANMessage(ACKNOWLEDGE_BASE_ROULANTE, 0, 0, 8, true, 0, 0, 0, 0, 0, 0, 0);
                sendCANMessage(ODO_SEND, 0, 0, 8, (((uint16_t)degrees(theta_robot) >> 8) & 0xFF), ((uint16_t)degrees(theta_robot) & 0xFF), (((uint16_t)odo_x >> 8) & 0xFF), ((uint16_t)odo_x & 0xFF), (((uint16_t)odo_y >> 8) & 0xFF), ((uint16_t)odo_y & 0xFF), 0, 0);
                liste.general_purpose = TYPE_DEPLACEMENT_IMMOBILE;
            }
            break;
        case TYPE_DEPLACEMENT_RECALAGE:
            Serial.printf(" TYPE_DEPLACEMENT_RECALAGE ");
            // TelnetStream.printf(" TYPE_DEPLACEMENT_RECALAGE ");

            if (recalage(liste.direction_recalage, liste.type_modif_x_y_theta_recalge_rien, liste.nouvelle_valeur_x_y_theta_rien, liste.consigne_rotation_recalge))
            {
                consigne_odo_droite_prec = odo_tick_droit;
                consigne_odo_gauche_prec = odo_tick_gauche;
                sendCANMessage(ACKNOWLEDGE_BASE_ROULANTE, 0, 0, 8, true, 0, 0, 0, 0, 0, 0, 0);
                liste.general_purpose = TYPE_DEPLACEMENT_IMMOBILE;
                // Serial.printf(" Odo x %.3f ", odo_x);
                // Serial.printf(" odo_y %.3f ", odo_y);
                // Serial.printf(" teheta %.3f ", degrees(theta_robot));
                // Serial.printf(" consigne_position_droite %.0f ", consigne_position_droite);
                // Serial.printf(" consigne_position_gauche %.0f ", consigne_position_gauche);
                // Serial.printf(" odo_tick_droit %.0f ", odo_tick_droit);
                // Serial.printf(" odo_tick_gauche %.0f ", odo_tick_gauche);
                // Serial.println();
            }
            // TelnetStream.println();

            break;
        case TYPE_VIDE:
            // Serial.printf(" TYPE_VIDE ");
            // Serial.printf(" Odo x %.3f ", odo_x);
            // Serial.printf(" odo_y %.3f ", odo_y);
            // Serial.printf(" teheta %.3f ", degrees(theta_robot));
            // Serial.println();
            break;

        case TYPE_EVITEMENT:

            break;

        default:
            break;
        }

        asservissement_roue_folle_droite_tick(consigne_position_droite, odo_tick_droit);
        asservissement_roue_folle_gauche_tick(consigne_position_gauche, odo_tick_gauche);
        // Serial.printf("caca");
        flag_controle = 1;
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(Te));
    }
}

void bus_can(void *parameters)
{

    TickType_t xLastWakeTime;
    xLastWakeTime = xTaskGetTickCount();
    while (1)
    {
        readCANMessage();

        // // Attendre la fin du mouvement avant de passer à l'ordre suivant
        // if (flag_fin_mvt)
        // {
        //     FIFO_lecture = (FIFO_lecture + 1) % SIZE_FIFO;
        //     flag_fin_mvt = false; // Réinitialiser le flag pour le prochain ordre

        //     // vTaskDelay(pdMS_TO_TICKS(Tcan)); // Temporisation pour éviter une boucle trop rapide
        //     // continue;                        // Retourne au début de la boucle en attendant que flag_fin_mvt soit vrai
        // }

        switch (rxMsg.id)
        {

        case ESP32_RESTART:
            Serial.println("ESP32_RESTART");
            liste.general_purpose = TYPE_VIDE;
            stop_motors();
            esp_restart();

            break;

        case ROTATION:
            // Pour se simplifier j'ai préférer décomposer les etapes quitte a prendre un peu plus de temps cpu
            enregistreur_odo();
            liste.general_purpose = TYPE_DEPLACEMENT_ROTATION;

            // liste.angle = TIC_PER_TOUR * angle / 80.0;

            liste.angle = convert_angle_deg_to_tick(fusion_octet(rxMsg.data[0], rxMsg.data[1]));
            liste.vitesse_croisiere = rxMsg.data[2];

            lauch_flag_asser_roue(true);
            rxMsg.id = 0;
            Serial.printf("ROTATION ");
            Serial.printf(" angle %f ", (float)fusion_octet(rxMsg.data[0], rxMsg.data[1]));
            Serial.printf(" liste.angle %f", (float)liste.angle);
            Serial.printf(" liste.vitesse_croisiere %d ", liste.vitesse_croisiere);

            // TelnetStream.printf("ROTATION ");
            // TelnetStream.printf(" angle %f ", (float)fusion_octet(rxMsg.data[0], rxMsg.data[1]));
            // TelnetStream.printf(" liste.angle %f", (float)liste.angle);
            // TelnetStream.printf(" liste.vitesse_croisiere %d ", liste.vitesse_croisiere);

            Serial.println();

            break;

        case LIGNE_DROITE:
            enregistreur_odo();
            liste.general_purpose = TYPE_DEPLACEMENT_LIGNE_DROITE;
            liste.distance = convert_distance_mm_to_tick(fusion_octet(rxMsg.data[0], rxMsg.data[1]));
            liste.vitesse_croisiere = rxMsg.data[2];
            lauch_flag_asser_roue(true);
            rxMsg.id = 0;

            Serial.printf("LIGNE_DROITE ");
            Serial.printf(" rxMsg.data[0] %d ", rxMsg.data[0]);
            Serial.printf(" rxMsg.data[1] %d ", rxMsg.data[1]);
            Serial.printf(" distance %d ", fusion_octet(rxMsg.data[0], rxMsg.data[1]));
            Serial.printf(" liste.distance %d ", liste.distance);
            Serial.printf(" liste.vitesse_croisiere %d ", liste.vitesse_croisiere);
            Serial.println();

            break;

        case POLAIRE:

            liste.nbr_passage = rxMsg.data[0];
            liste.x_polaire[liste.nbr_passage] = fusion_octet(rxMsg.data[1], rxMsg.data[2]);
            liste.y_polaire[liste.nbr_passage] = fusion_octet(rxMsg.data[3], rxMsg.data[4]);

            liste.checksum_nbr_passage = rxMsg.data[5] - 1;
            liste.rotation_polaire[liste.nbr_passage] = fusion_octet(rxMsg.data[6], rxMsg.data[7]);

            if (liste.checksum_nbr_passage == liste.nbr_passage)
            {
                enregistreur_odo();
                // Serial.printf("fâpkfakfa^pkfaêfpk^,ae^c,");
                liste.general_purpose = TYPE_DEPLACEMENT_X_Y_POLAIRE;
                flag_fin_mvt = false;
                rxMsg.id = 0;
                liste.compteur_point_de_passage_polaire = 0;
            }

            Serial.printf(" POLAIRE ");
            Serial.printf(" liste.nbr_passage %d ", liste.nbr_passage);
            Serial.printf(" liste.checksum_nbr_passage %d ", liste.checksum_nbr_passage);

            Serial.printf(" liste.x_polaire %f ", liste.x_polaire[liste.nbr_passage]);
            Serial.printf(" liste.y_polaire %f ", liste.y_polaire[liste.nbr_passage]);
            Serial.printf(" liste.cptps %d ", liste.compteur_point_de_passage_polaire);

            TelnetStream.printf(" POLAIRE ");
            TelnetStream.printf(" liste.nbr_passage %d ", liste.nbr_passage);
            TelnetStream.printf(" liste.checksum_nbr_passage %d ", liste.checksum_nbr_passage);
            TelnetStream.printf(" liste.x_polaire %f ", liste.x_polaire[liste.nbr_passage]);
            TelnetStream.printf(" liste.y_polaire %f ", liste.y_polaire[liste.nbr_passage]);
            // TelnetStream.printf(" liste.cptps %d ", liste.compteur_point_de_passage_polaire);

            // Serial.printf(" rxMsg.data[1] %d ", rxMsg.data[1]);
            // Serial.printf(" rxMsg.data[2] %d ", rxMsg.data[2]);
            // Serial.printf(" rxMsg.data[3] %d ", rxMsg.data[3]);
            // Serial.printf(" rxMsg.data[4] %d ", rxMsg.data[4]);

            Serial.println();
            TelnetStream.println();

            break;
        case RECALAGE:
            enregistreur_odo();

            liste.general_purpose = TYPE_DEPLACEMENT_RECALAGE;
            liste.direction_recalage = rxMsg.data[0];
            liste.type_modif_x_y_theta_recalge_rien = rxMsg.data[1];
            liste.nouvelle_valeur_x_y_theta_rien = fusion_octet(rxMsg.data[2], rxMsg.data[3]);
            liste.consigne_rotation_recalge = convert_angle_deg_to_tick(fusion_octet(rxMsg.data[4], rxMsg.data[5]));

            rxMsg.id = 0;
            Serial.printf(" RECALAGE ");
            Serial.printf(" liste.direction_recalage %d ", liste.direction_recalage);
            Serial.printf(" liste.type_modif_x_y_theta_recalge_rien %d ", liste.type_modif_x_y_theta_recalge_rien);
            Serial.printf(" liste.nouvelle_valeur_x_y_theta_rien %d ", liste.nouvelle_valeur_x_y_theta_rien);
            Serial.printf(" liste.consigne_rotation_recalge %d ", liste.consigne_rotation_recalge);

            TelnetStream.printf(" RECALAGE ");
            TelnetStream.printf(" liste.direction_recalage %d ", liste.direction_recalage);
            TelnetStream.printf(" liste.type_modif_x_y_theta_recalge_rien %d ", liste.type_modif_x_y_theta_recalge_rien);
            TelnetStream.printf(" liste.nouvelle_valeur_x_y_theta_rien %d ", liste.nouvelle_valeur_x_y_theta_rien);
            TelnetStream.printf(" liste.consigne_rotation_recalge %d ", liste.consigne_rotation_recalge);
            Serial.println();

            break;
            // case STOP_ROBOT_FIN_MATCH:
            //     stop_start_robot_fin_match = rxMsg.data[0];

            //     break;
            // case START_ROBOT_MATCH:
            //     stop_start_robot_fin_match = rxMsg.data[0];

            //     break;

        case CELLULE_BAT:
            // Ne sert a rien
            break;
        case BATT_MAIN:

            Serial.printf("BATT_Main ");
            Serial.printf(" tension %.2f V", conversion_4char_to_float(&rxMsg.data[0]));
            Serial.printf(" courant %.2f mA", conversion_4char_to_float(&rxMsg.data[4]));
            Serial.println();

            rxMsg.id = 0;

            break;
        case BATT_1:
            tension = conversion_4char_to_float(&rxMsg.data[0]);
            courant = conversion_4char_to_float(&rxMsg.data[4]);

            if (tension > 9.5)
            {
                rectificateur_coeeff = 13.0 / tension;
                Serial.printf("BATT_1 ");
                Serial.printf(" tension %.2f V", tension);
                Serial.printf(" courant %.2f mA", courant);
                Serial.println();
            }
            rxMsg.id = 0;

            break;
        case BATT_2:
            tension = conversion_4char_to_float(&rxMsg.data[0]);
            courant = conversion_4char_to_float(&rxMsg.data[4]);

            if (tension > 9.5)
            {
                rectificateur_coeeff = 13.0 / tension;
                Serial.printf(" BATT_2 ");
                Serial.printf(" tension %.2f V", tension);
                Serial.printf(" courant %.2f mA", courant);
                Serial.println();
            }

            // Serial.println();
            rxMsg.id = 0;

            break;
        case BATT_3:

            tension = conversion_4char_to_float(&rxMsg.data[0]);
            courant = conversion_4char_to_float(&rxMsg.data[4]);

            if (tension > 9.5)
            {
                rectificateur_coeeff = 13.0 / tension;
                Serial.printf(" BATT_3 ");
                Serial.printf(" tension %.2f V", tension);
                Serial.printf(" courant %.2f mA", courant);
                Serial.println();
            }

            rxMsg.id = 0;

            break;

        case INTERRUPTEUR_BATT1:

            Serial.printf(" INTERRUPTEUR_BATT1 ");
            Serial.println();
            rxMsg.id = 0;

            break;
        case INTERRUPTEUR_BATT2:

            Serial.printf(" INTERRUPTEUR_BATT2 ");
            Serial.println();
            rxMsg.id = 0;

            break;
        case INTERRUPTEUR_BATT3:

            Serial.printf("  INTERRUPTEUR_BATT3 ");
            Serial.println();
            rxMsg.id = 0;

            break;

        case 0:
            // Serial.printf("Rien\n");
            // {
            //     int16_t odo_x_int = static_cast<int16_t>(odo_x * 10);
            //     int16_t odo_y_int = static_cast<int16_t>(odo_y * 10);

            //     uint8_t lowByte_x = odo_x_int & 0xFF;
            //     uint8_t highByte_x = (odo_x_int >> 8) & 0xFF;
            //     uint8_t lowByte_y = odo_y_int & 0xFF;
            //     uint8_t highByte_y = (odo_y_int >> 8) & 0xFF;
            //     // sendCANMessage(ODO_SEND, 0, 0, 8, highByte_x, lowByte_x, highByte_y, lowByte_y, 0, 0, 0, 0);
            // }

            break;

        default:
            break;
        }

        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(Tcan));
    }
}

void setup()
{ // calcul coeff filtre
  // delay(10000);
  // Initialisation de la communication série à 115200 bauds
    // odo_x = 1225;
    // odo_y = 139.0;
    Serial.begin(115200);
    // Serial.println("Booting with OTA"); // Message indiquant le démarrage avec OTA
    // Appel à la fonction de configuration OTA (non définie dans ce code, mais probablement ailleurs)
    // setupOTA();
    // Initialisation des moteurs
    setup_motors();
    stop_motors();
    // Initialisation des encodeurs
    setup_encodeur();
    // Initialisation du Can
    setupCAN(1000E3);

    // Boucle jusqu'à ce qu'un client soit connecté via le port série WiFi
    // while (!SerialWIFI.available())
    // {
    //     delay(500);                                             // Attente de 500 ms avant de vérifier à nouveau
    //     Serial.println("Aucun client connecté, en attente..."); // Message indiquant qu'il n'y a pas de client connecté
    // }
    // delay(10000);
    // affichage_commande_wifi();
    Serial.println("on commence");

    // Serial.printf("avncement_gauche enter : %.0f\n", avncement_gauche);
    // Serial.printf("avncement_droite enter : %.0f\n", avncement_droite);

    reset_encodeur();
    delay(1000);
    reset_encodeur();

    xTaskCreate(
        controle,   // nom de la fonction
        "controle", // nom de la tache que nous venons de vréer
        10000,      // taille de la pile en octet
        NULL,       // parametre
        10,         // tres haut niveau de priorite
        NULL        // descripteur
    );
    // xTaskCreate(
    //     odo,   // nom de la fonction
    //     "odo", // nom de la tache que nous venons de vréer
    //     10000, // taille de la pile en octet
    //     NULL,  // parametre
    //     11,    // tres haut niveau de priorite
    //     NULL   // descripteur
    // );
    xTaskCreate(
        bus_can,   // nom de la fonction
        "bus_can", // nom de la tache que nous venons de vréer
        10000,     // taille de la pile en octet
        NULL,      // parametre
        9,         // tres haut niveau de priorite
        NULL       // descripteur
    );
}

// Boucle principale, exécutée en continu après le setup
void loop()
{
    if (flag_controle)
    {
        if (flag_fin_mvt == true)
        {
            // Serial.printf(" PS_ASSER %d ", pause_asser_test);

            // Serial.printf(" Odo x %.3f ", odo_x);
            // Serial.printf(" odo_y %.3f ", odo_y);
            // Serial.printf(" teheta %.3f ", degrees(theta_robot));
            // // Serial.printf("CPT_PS %d", liste.compteur_point_de_passage_polaire);

            // Serial.printf(" er_d %.3f ", convert_distance_tick_to_mm(erreur_distance));
            // Serial.printf(" er_o %.3f ", convert_tick_to_angle_deg(erreur_orient));
            // Serial.printf(" delta_droit %.0f ", delta_droit);
            // Serial.printf("ROTATION ");
            // Serial.printf(" angle %f ", (float)fusion_octet(rxMsg.data[0], rxMsg.data[1]));
            // Serial.printf(" liste.dist %f", (float)liste.distance);

            // Serial.printf(" consigne_position_droite %.0f ", consigne_position_droite);
            // Serial.printf(" consigne_position_gauche %.0f ", consigne_position_gauche);

            // Serial.printf(" odo_tick_droit %.0f ", odo_tick_droit);
            // Serial.printf(" odo_tick_gauche %.0f ", odo_tick_gauche);

            // Serial.printf(" delta_tick_droit %.0f ", delta_odo_tick_droit);
            // Serial.printf(" delta_tick_gauche %.0f ", delta_odo_tick_gauche);

            // Serial.printf("cs_dist_mm %f", convert_distance_tick_to_mm(liste.distance));
            // Serial.printf(" cs_dist_tic %d", (liste.distance));

            // // // Serial.printf(" vitesse robo %f ", vitesse_rob);

            // // Serial.printf(" etat_x_y_theta x %d ", etat_x_y_theta);
            // Serial.print("Etat actuel : " + toStringG(etat_actuel_vit_roue_folle_gauche));
            // Serial.print(" " + toStringD(etat_actuel_vit_roue_folle_droite));
            // Serial.println();
        }
        flag_controle = 0;
    }
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
