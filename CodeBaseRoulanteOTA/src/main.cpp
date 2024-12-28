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

int etat_mouvement = 0;
int cons_distance_ticks = 5000;
int cons_rotation_ticks = 2250;
int vitesse_ligne_droite = 70;
int vitesse_rot = 120;
int time_wait = 5000;
bool inverter_mouv_order = 0;
float seuil_recalage = 500;
// Fonction pour convertir un état en texte
String toStringG(Etat_vitesse_roue_folle_gauche etat)
{
    switch (etat)
    {
    case ETAT_ATTENTE_Vitesse_ROUE_FOLLE_GAUCHE:
        return "ETAT_ATTENTE_Vitesse_ROUE_FOLLE_GAUCHE";

    case ETAT_ACCELERATION_Vitesse_ROUE_FOLLE_GAUCHE:
        return "ETAT_ACCELERATION_Vitesse_ROUE_FOLLE_GAUCHE";
    case ETAT_CROISIERE_Vitesse_ROUE_FOLLE_GAUCHE:
        return "ETAT_CROISIERE_Vitesse_ROUE_FOLLE_GAUCHE";
    case ETAT_DECELERATION_Vitesse_ROUE_FOLLE_GAUCHE:
        return "ETAT_DECELERATION_Vitesse_ROUE_FOLLE_GAUCHE";
    case ETAT_ARRET_Vitesse_ROUE_FOLLE_GAUCHE:
        return "ETAT_ARRET_Vitesse_ROUE_FOLLE_GAUCHE";
    case ETAT_VIDE_Vitesse_ROUE_FOLLE_GAUCHE:
        return "ETAT_VIDE_Vitesse_ROUE_FOLLE_GAUCHE";
    default:
        return "ETAT_INCONNU";
    }
}
String toStringD(Etat_vitesse_roue_folle_droite etat)
{
    switch (etat)
    {
    case ETAT_ATTENTE_Vitesse_ROUE_FOLLE_DROITE:
        return "ETAT_ATTENTE_Vitesse_ROUE_FOLLE_DROITE";
    case ETAT_ACCELERATION_Vitesse_ROUE_FOLLE_DROITE:
        return "ETAT_ACCELERATION_Vitesse_ROUE_FOLLE_DROITE";
    case ETAT_CROISIERE_Vitesse_ROUE_FOLLE_DROITE:
        return "ETAT_CROISIERE_Vitesse_ROUE_FOLLE_DROITE";
    case ETAT_DECELERATION_Vitesse_ROUE_FOLLE_DROITE:
        return "ETAT_DECELERATION_Vitesse_ROUE_FOLLE_DROITE";
    case ETAT_ARRET_Vitesse_ROUE_FOLLE_DROITE:
        return "ETAT_ARRET_Vitesse_ROUE_FOLLE_DROITE";
    case ETAT_VIDE_Vitesse_ROUE_FOLLE_DROITE:
        return "ETAT_VIDE_Vitesse_ROUE_FOLLE_DROITE";
    default:
        return "ETAT_INCONNU";
    }
}

struct Ordre_deplacement
{
    int general_purpose;
    int angle;
    int sens_rotation;
    double distance;
    int vitesse_croisiere;
    int sens_ligne_droite;
    int consigne_distance_recalage;
    int vitesse_recalage;
    int sens_recalage;
    int x;
    int y;
    int theta;
    int vitesse_x_y_theta;
};
Ordre_deplacement liste = {TYPE_DEPLACEMENT_X_Y_THETA, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

void controle(void *parameters)
{
    TickType_t xLastWakeTime;
    xLastWakeTime = xTaskGetTickCount();
    while (1)
    {

        switch (liste.general_purpose)
        {
        case TYPE_DEPLACEMENT_LIGNE_DROITE:

            ligne_droite(liste.distance, liste.vitesse_croisiere, liste.sens_ligne_droite);
            // Serial.printf("TYPE_DEPLACEMENT_LIGNE_DROITE");

            if (return_flag_asser_roue())
            {
                flag_fin_mvt = true;
                sendCANMessage(ACKNOWLEDGE_BASE_ROULANTE, 0, 0, 1, flag_fin_mvt, TYPE_DEPLACEMENT_LIGNE_DROITE, 0, 0, 0, 0, 0);
                liste.general_purpose = TYPE_DEPLACEMENT_IMMOBILE;
            }
            break;
        case TYPE_DEPLACEMENT_ROTATION:

            // Serial.printf("TYPE_DEPLACEMENT_ROTATION");
            rotation(liste.angle, liste.vitesse_croisiere, liste.sens_rotation);

            if (return_flag_asser_roue())
            {
                flag_fin_mvt = true;
                sendCANMessage(ACKNOWLEDGE_BASE_ROULANTE, 0, 0, 1, flag_fin_mvt, TYPE_DEPLACEMENT_ROTATION, 0, 0, 0, 0, 0);
                liste.general_purpose = TYPE_DEPLACEMENT_IMMOBILE;
            }
            break;
        case TYPE_DEPLACEMENT_IMMOBILE:
            consigne_regulation_vitesse_droite = consigne_odo_droite_prec;
            consigne_regulation_vitesse_gauche = consigne_odo_gauche_prec;
            // Serial.printf(" TYPE_DEPLACEMENT_IMMOBILE ");
            liste.general_purpose = TYPE_VIDE;
            flag_fin_mvt = true;
            sendCANMessage(ACKNOWLEDGE_BASE_ROULANTE, 0, 0, 1, flag_fin_mvt, TYPE_DEPLACEMENT_IMMOBILE, 0, 0, 0, 0, 0);

            break;
        case TYPE_DEPLACEMENT_X_Y_THETA:
            // Serial.print("Etat actuel : " + toStringG(etat_actuel_vit_roue_folle_gauche));
            // Serial.println(" " + toStringD(etat_actuel_vit_roue_folle_droite));

            x_y_theta(liste.x, liste.y, liste.theta, liste.vitesse_x_y_theta);

            if (etat_x_y_theta == -1)
            {
                flag_fin_mvt = true;
                sendCANMessage(ACKNOWLEDGE_BASE_ROULANTE, 0, 0, 1, flag_fin_mvt, TYPE_DEPLACEMENT_X_Y_THETA, 0, 0, 0, 0, 0);
                liste.general_purpose = TYPE_DEPLACEMENT_IMMOBILE;
            }
            break;
        case TYPE_VIDE:
            // Serial.printf(" TYPE_VIDE \n");
            break;

        default:
            break;
        }
        asservissement_roue_folle_droite_tick(consigne_regulation_vitesse_droite, odo_tick_droit);
        asservissement_roue_folle_gauche_tick(consigne_regulation_vitesse_gauche, odo_tick_gauche);
        Serial.printf(" Odo x %.3f ", odo_x);
        Serial.printf(" odo_y %.3f ", odo_y);
        Serial.printf(" teheta %.3f ", degrees(theta_robot));
        Serial.println();
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(Te));
    }
}

void odo(void *parameters)
{
    TickType_t xLastWakeTime;
    xLastWakeTime = xTaskGetTickCount();
    while (1)
    {

        read_x_y_theta();

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
        {
            flag_fin_mvt = false;
            // Pour se simplifier j'ai préférer décomposer les etapes quitte a prendre un peu plus de temps cpu
            int16_t angle = fusion_octet(rxMsg.data[0], rxMsg.data[1]);
            int8_t sens_rotation = rxMsg.data[2];
            int8_t vitesse = rxMsg.data[3];

            liste.general_purpose = TYPE_DEPLACEMENT_ROTATION;
            float distance_a_faire_en_mm = angle * perimetre_robot / 360;
            float consigne_roue_odo = distance_a_faire_en_mm * (TIC_PER_TOUR / (2 * M_PI * SIZE_WHEEL_DIAMETER_mm / 2.0));
            liste.angle = consigne_roue_odo;
            // liste.angle = TIC_PER_TOUR * angle / 80.0;

            liste.sens_rotation = sens_rotation;
            liste.vitesse_croisiere = vitesse;
            lauch_flag_asser_roue(true);
            rxMsg.id = 0;
            // Serial.printf("ROTATION ");
            // Serial.printf(" angle %f ", (float)angle);
            // Serial.printf(" liste.angle %f", (float)liste.angle);
            // Serial.println();
            // Serial.printf(" sens_rotation %d ", liste.sens_rotation);
            // Serial.printf(" liste.vitesse_croisiere %d ", liste.vitesse_croisiere);
            // Serial.println();
        }
        break;

        case LIGNE_DROITE:

        {
            flag_fin_mvt = false;
            // Pour se simplifier j'ai préférer décomposer les etapes quitte a prendre un peu plus de temps cpu
            double distance = fusion_octet(rxMsg.data[0], rxMsg.data[1]);
            int8_t sens_ligne_droite = rxMsg.data[2];
            int8_t vitesse = rxMsg.data[3];

            liste.general_purpose = TYPE_DEPLACEMENT_LIGNE_DROITE;
            liste.distance = distance * (TIC_PER_TOUR / (2 * M_PI * SIZE_WHEEL_DIAMETER_mm / 2.0));
            liste.sens_ligne_droite = sens_ligne_droite;
            liste.vitesse_croisiere = vitesse;
            lauch_flag_asser_roue(true);

            rxMsg.id = 0;

            // Serial.printf("LIGNE_DROITE ");
            // Serial.printf(" distance %f ", distance);
            // Serial.printf(" liste.distance %f ", liste.distance);
            // Serial.printf(" liste.sens_ligne_droite %d ", liste.sens_ligne_droite);
            // Serial.printf(" liste.vitesse_croisiere %d ", liste.vitesse_croisiere);
            // Serial.println();
        }

        break;
        case 0:

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
    Serial.begin(115200);
    // Serial.println("Booting with OTA"); // Message indiquant le démarrage avec OTA
    // Appel à la fonction de configuration OTA (non définie dans ce code, mais probablement ailleurs)
    setupOTA();
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
    // delay(1000);
    reset_encodeur();

    xTaskCreate(
        controle,   // nom de la fonction
        "controle", // nom de la tache que nous venons de vréer
        10000,      // taille de la pile en octet
        NULL,       // parametre
        10,         // tres haut niveau de priorite
        NULL        // descripteur
    );
    xTaskCreate(
        odo,   // nom de la fonction
        "odo", // nom de la tache que nous venons de vréer
        10000, // taille de la pile en octet
        NULL,  // parametre
        11,    // tres haut niveau de priorite
        NULL   // descripteur
    );
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
} 