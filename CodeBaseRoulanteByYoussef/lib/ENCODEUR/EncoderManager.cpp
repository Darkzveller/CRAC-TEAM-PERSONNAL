// EncoderManager.cpp
#include "../ID.h" // Remonte d'un niveau pour atteindre lib
#include "EncoderManager.h"

extern float Te;

float Echantillon_ms_precedent = 0;
const unsigned long Intervalle_T = Te;

// Constructeur avec les pins des encodeurs gauche et droit
EncoderManager::EncoderManager(int pinA, int pinB)
    : pinA(pinA), pinB(pinB) {}

// Méthode pour initialiser les encodeurs
void EncoderManager::init(String nameEncoder, float wheel_size_mm, float TIC_ONE_TOUR)
{
    nameEncodeur = nameEncoder;
    TIC_UN_TOUR = TIC_ONE_TOUR;
    ESP32Encoder::useInternalWeakPullResistors = UP; // Utilise les résistances internes
    encoder.attachHalfQuad(pinA, pinB);
    encoder.clearCount();

    if (DEBUG_ENCODEUR_INIT)
    {
        Serial.printf(" %s à été initialisé", nameEncodeur.c_str());
        Serial.println();
    }
}

long EncoderManager::getTickPosition()
{
    long val_encodeur = encoder.getCount();
    if (DEBUG_ENCODEUR_TICK)
    {
        Serial.printf("Le %s  %d", nameEncodeur.c_str(), val_encodeur);
        Serial.println();
    }
    return val_encodeur;
}

float EncoderManager::getDistance()
{
    long val_tick = EncoderManager::getTickPosition();
    float number_tour = val_tick * 1.0 / TIC_UN_TOUR;
    if (DEBUG_ENCODEUR_DISTANCE)
    {
        Serial.printf("Le %s a fait %f tour", nameEncodeur.c_str(), number_tour);
        Serial.println();
    }
    return number_tour;
}

float EncoderManager::getAngle()
{
    float val_distance = EncoderManager::getDistance();
    float angle_radians = 2.0 * M_PI * val_distance / 1.0;
    if (DEBUG_ENCODEUR_Angle)
    {
        Serial.printf("Le %s a fait %f radians", nameEncodeur.c_str(), angle_radians);
        Serial.println();
    }
    return angle_radians;
}

float EncoderManager::getVitesse()
{
    float angle_actuelle = EncoderManager::getAngle(), vitesse;
    // float angle_actuelle = EncoderManager::getAngle();
    // float vitesse_mm_par_s = (angle_actuelle-angle_precedent)/Te;
    // if (DEBUG_ENCODEUR_Angle)
    // {
    //     Serial.printf("Le %s a fait %f radians", nameEncodeur.c_str(), angle_radians);
    //     Serial.println();
    // }
    if (millis() >= (Echantillon_ms_precedent + Intervalle_T))
    {
        Echantillon_ms_precedent = millis();
        angle_actuelle = EncoderManager::getAngle();
        vitesse = (angle_actuelle - angle_precedent) / Te;
        angle_precedent = angle_actuelle;
        if (DEBUG_ENCODEUR_VITESSE)
        {
            Serial.printf("Angle actu %f  angle prec %f", angle_actuelle, angle_precedent);
            Serial.printf(" Vitesse %f en mm par ms", vitesse);
            Serial.println();
        }
    }
    return vitesse;
}

void EncoderManager::showTickPosition()
{
    long val_encodeur = encoder.getCount();

    Serial.printf("Le %s  %d", nameEncodeur.c_str(), val_encodeur);
    Serial.println();
}
// Je suis partie du principe que ca fonctionner :)
void EncoderManager::resetPosition()
{
    encoder.clearCount();
    if (DEBUG_ENCODEUR_RESET)
    {
        Serial.printf(" %s à été reset", nameEncodeur.c_str());
        Serial.println();
    }
}
