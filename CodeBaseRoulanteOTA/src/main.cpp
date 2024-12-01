#include "OTA.h" // Inclusion de la bibliothèque OTA (Over-The-Air) pour les mises à jour sans fil
#include "Variable.h"
#include "MOTEUR.h"
#include "EncoderManager.h"
#include "ASSERVISSEMENT.h"
#include "MOUVEMENT.h"
#include <mat.h>

bool stop_asservissement_roue_gauche = 0;
bool stop_asservissement_roue_droite = 0;
bool start_asservissement_roue_gauche = 0;
bool start_asservissement_roue_droite = 0;

float consigne_odo_droite_delta = 0;
float consigne_odo_gauche_delta = 0;
extern float offset;

int state = 0;
int cons_distance_ticks = 5000;
int cons_rotation_ticks = 2250;
int time_wait = 5000;
bool inverter_mouv_order = 1;
// Fonction pour convertir un état en texte
String toString(Etat_vitesse_roue_folle_droite etat)
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

void controle(void *parameters)
{
    TickType_t xLastWakeTime;
    xLastWakeTime = xTaskGetTickCount();
    while (1)
    {
        /**/
        switch (state)
        {
        case 0:
            if (inverter_mouv_order)
            {
                rotation(cons_rotation_ticks, 100, 1);
            }
            else
            {
                ligne_droite(cons_distance_ticks, 100, 1);
            }
            if ((start_asservissement_roue_droite == false) && (start_asservissement_roue_gauche == false))
            {
                state = 1;
                start_asservissement_roue_droite = true;
                start_asservissement_roue_gauche = true;
            }
            break;
        case 1:
            Serial.printf(" consigne_odo_gauche_prec %.0f ", consigne_odo_gauche_prec);
            Serial.printf(" consigne_odo_droite_prec %.0f ", consigne_odo_droite_prec);

            if (inverter_mouv_order)
            {
                ligne_droite(cons_distance_ticks , 40, 1);
            }
            else
            {
                rotation(cons_rotation_ticks, 100, 1);
            }
            if ((start_asservissement_roue_droite == false) && (start_asservissement_roue_gauche == false))
            {
                state = 2;
            }
            break;
        case 2:
            Serial.print(" FONO ");
            // stop_motors();
            // consigne_regulation_vitesse_droite = consigne_odo_droite_prec;
            // consigne_regulation_vitesse_gauche = consigne_odo_gauche_prec;
            break;

        default:
            break;
        }
        asservissement_roue_folle_droite_tick(consigne_regulation_vitesse_droite, odo_tick_droit);
        asservissement_roue_folle_gauche_tick(consigne_regulation_vitesse_gauche, odo_tick_gauche);

        /*
       rotation((2250 * 1), 70, -1);
       // ligne_droite(+30000, 135, 1);
       if ((flag_controle = 1) == 1)
       {
           asservissement_roue_folle_droite_tick(consigne_regulation_vitesse_droite, odo_tick_droit);
           asservissement_roue_folle_gauche_tick(consigne_regulation_vitesse_gauche, odo_tick_gauche);
       }
       else
       {
           stop_motors();
       }
       */
        // Serial.printf("obs %4.0f", observation);

        Serial.printf("| odo gauche %.0f odo droite %.0f", odo_tick_gauche, odo_tick_droit);
        // Serial.printf("| consigne_regulation_vitesse_droite %.0f consigne_regulation_vitesse_gauche_rec  %.0f", consigne_regulation_vitesse_droite, consigne_regulation_vitesse_gauche);
        Serial.printf(" Theta %3.1f ", theta_robot * 180 / 3.14);
        // Serial.printf("nmbr tour %2.3f", (double)(theta_robot * 180 / M_PI / 360));
        Serial.println("Etat actuel : " + toString(etat_actuel_vit_roue_folle_droite));
        //   Serial.println();
        // delay(1000);
        // FlagCalcul = 1;
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
    // Initialisation des encodeurs
    setup_encodeur();

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
    start_asservissement_roue_droite = true;
    start_asservissement_roue_gauche = true;
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
}

// Boucle principale, exécutée en continu après le setup
void loop()
{
}
