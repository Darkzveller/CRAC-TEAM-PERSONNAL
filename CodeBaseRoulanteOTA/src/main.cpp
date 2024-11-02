#include "OTA.h" // Inclusion de la bibliothèque OTA (Over-The-Air) pour les mises à jour sans fil
#include "Variable.h"
#include "MOTEUR.h"
#include "EncoderManager.h"
#include "ASSERVISSEMENT.h"
#include <mat.h>

float consigne_regulation_vitesse_droite = 0;
float consigne_regulation_vitesse_gauche = 0;
float sommme_integral_regulation_vitesse = 0;
float integral_limit_regulation_vitesse = 1000;
float erreur_precedente_regulation_vitesse = 0;

float coeff_P = 0.25;
float coeff_D = 0;
float coeff_I = 0;

float nbr_tour = 90 / 3600 * 3600 / 90;
// float avncement_droite = +4254 / 2 * nbr_tour; // Distance en ticks (ajuster selon tes besoins)
// float avncement_gauche = +4254 / 2 * nbr_tour; // Distance en ticks (ajuster selon tes besoins)
float avncement_droite = -2250*2;
float avncement_gauche = -2250*2;
bool stop = 0;
float vitesse_croisiere = 35;
float vitesse_croisiere_droit = vitesse_croisiere;
float vitesse_croisiere_gauche = vitesse_croisiere;
float emballement = 1.15;

void controle(void *parameters)
{
    TickType_t xLastWakeTime;
    xLastWakeTime = xTaskGetTickCount();
    while (1)
    {
/*
        double sortie;
        double observation = consigne_regulation_vitesse_gauche + consigne_regulation_vitesse_droite;
        double erreur = 0 - observation;
        double proportionnel = erreur * coeff_P;

        double deriver = coeff_D * (erreur - erreur_prec_roue_folle_droite_tick) / Te;

        sommme_integral_regulation_vitesse += erreur * Te;
        if (sommme_integral_regulation_vitesse > integral_limit_regulation_vitesse)
        {
            sommme_integral_regulation_vitesse = integral_limit_regulation_vitesse;
        }
        else if (sommme_integral_regulation_vitesse < -integral_limit_regulation_vitesse)
        {
            sommme_integral_regulation_vitesse = -integral_limit_regulation_vitesse;
        }

        double integral = coeff_I * sommme_integral_regulation_vitesse;

        double commande = proportionnel + deriver + integral;

        erreur_precedente_regulation_vitesse = erreur;

        // Gestion des bornes de la commande
        float saturation = 1000;
        if (commande > 0)
        {

            if (commande > (saturation))
            {
                sortie = (saturation);
            }
            else
            {
                sortie = commande;
            }
        }
        else
        {
            if (commande < -(saturation))
            {

                sortie = -(saturation);
            }
            else
            {
                sortie = -commande;
            }
        }*/
        // consigne_regulation_vitesse_gauche = consigne_regulation_vitesse_gauche + sortie;
        // float vitesse_croisiere_gauche =  vitesse_croisiere+ sortie;
        consigne_regulation_vitesse_droite = regulation_vitesse_roue_folle_droite((avncement_droite), vitesse_croisiere_droit);
        consigne_regulation_vitesse_gauche = regulation_vitesse_roue_folle_gauche((avncement_gauche), vitesse_croisiere_gauche);
        // consigne_regulation_vitesse_gauche = -consigne_dist_droite;
        // Serial.printf("cmd %5.2f err %5.2f obs %5.2f  consigne_croisiere_droit %5.2f consigne_croisiere_gauche %4.0f ", commande, erreur, (observation), consigne_regulation_vitesse_droite, consigne_regulation_vitesse_gauche);

        if ((flag_controle = 1) == 1)
        {
            asservissement_roue_folle_droite_tick(consigne_regulation_vitesse_droite, odo_tick_droit);
            asservissement_roue_folle_gauche_tick(consigne_regulation_vitesse_gauche, odo_tick_gauche);
        }
        else
        {
            stop_motors();
        }
        // Serial.printf("obs %4.0f", observation);
        Serial.printf("| odo gauche %.0f odo droite %.0f", odo_tick_gauche, odo_tick_droit);
        SerialWIFI.printf("|consigne_regulation_vitesse_droite %5.2f consigne_regulation_vitesse_gauche %5.2f ", consigne_regulation_vitesse_droite, consigne_regulation_vitesse_gauche);
        // SerialWIFI.printf("| consigne_regulation_vitesse_droite %.0f consigne_regulation_vitesse_gauche  %.0f |", consigne_regulation_vitesse_droite, consigne_regulation_vitesse_gauche);
        Serial.printf(" Theta %3.1f ", theta_robot * 180 / 3.14);
        Serial.printf("nmbr tour %2.0f", (double)theta_robot * 180 / 3.14 / 360);
        Serial.println();
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

        // FlagCalcul = 1;
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
    Serial.printf("avncement_gauche enter : %.0f\n", avncement_gauche);
    Serial.printf("avncement_droite enter : %.0f\n", avncement_droite);
    if (avncement_droite > 0)
    {
        vitesse_croisiere_droit = vitesse_croisiere_droit;
    }
    else if (avncement_droite < 0)
    {
        vitesse_croisiere_droit = -vitesse_croisiere_droit;
    }
    if (avncement_gauche > 0)
    {
        vitesse_croisiere_gauche = vitesse_croisiere_gauche;
    }
    else if (avncement_gauche < 0)
    {
        vitesse_croisiere_gauche = -vitesse_croisiere_gauche;
    }
    reset_encodeur();
    delay(5000);
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
}

// Boucle principale, exécutée en continu après le setup
void loop()
{
}
