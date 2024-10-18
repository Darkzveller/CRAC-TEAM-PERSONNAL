#include "../Variable.h" // Remonte d'un niveau pour atteindre lib
#include <math.h>
#include "MOTEUR.h"

void setup_motors()
{
    // Moteur droit
    pinMode(M1_INA, OUTPUT);
    pinMode(M1_INB, OUTPUT);
    ledcSetup(channel_1, frequence, resolution);
    ledcAttachPin(PWM_1, channel_1);
    // Moteur Gauche
    pinMode(M2_INA, OUTPUT);
    pinMode(M2_INB, OUTPUT);
    // ledcSetup(channel_2, frequence, resolution);
    ledcAttachPin(PWM_2, channel_2);
}

void moteur_droit(int pwm, bool sens)
{
    if (sens == true)
    {
        digitalWrite(M1_INA, 1);
        digitalWrite(M1_INB, 0);
    }
    if (sens == false)
    {
        digitalWrite(M1_INA, 0);
        digitalWrite(M1_INB, 1);
    }
    ledcWrite(channel_1, pwm);
}

void moteur_gauche(int pwm, bool sens)
{
    if (sens == true)
    {
        digitalWrite(M2_INA, 1);
        digitalWrite(M2_INB, 0);
    }

    if (sens == false)
    {
        digitalWrite(M2_INA, 1);
        digitalWrite(M2_INB, 0);
    }
    ledcWrite(channel_2, pwm);
}

void stop_motors()
{

    digitalWrite(M2_INA, 0);
    digitalWrite(M2_INB, 0);

    digitalWrite(M1_INA, 0);
    digitalWrite(M1_INB, 0);
}
