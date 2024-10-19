#include "Variable.h" // Remonte d'un niveau pour atteindre lib
#include "MOTEUR.h"
#include <math.h>

void setup_motors()
{
    // Moteur droit
    pinMode(M1_INA, OUTPUT);
    pinMode(M1_INB, OUTPUT);
    ledcSetup(channel_1, frequence, resolution_pwm);
    ledcAttachPin(PWM_1, channel_1);
    // Moteur gauche
    pinMode(M2_INA, OUTPUT);
    pinMode(M2_INB, OUTPUT);
    ledcAttachPin(PWM_2, channel_2);
}

void moteur_droit(int pwm, bool sens)
{
    if (sens == true)
    {
        digitalWrite(M1_INA, 1);
        digitalWrite(M1_INB, 0);
    }
    else
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
    else
    {
        digitalWrite(M2_INA, 0);
        digitalWrite(M2_INB, 1);
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
