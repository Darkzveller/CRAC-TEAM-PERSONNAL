#include <Arduino.h>

#ifndef _MOTEUR_H
#define _MOTEUR_H

#define COMMANDE_UNIPOLAIRE
// #define COMMANDE_BIPOLAIRE

class moteur
{

private:
    int speed = 0, deltaSpeed = 0;
    int pinA, pinB;
    int channelA, channelB;
    int resolution;
   double saturation_pwm_pourcent;
    String nameMotor;

public:
    // Initialise le moteur avec le pin, le canal souhaiter, la fréquence souhaiter et la résolution
    void init(int PINA, int PINB, int channel_pwm, int frequence, int resolution_bits, String Name_Motor);
    void setSpeed(int motorSpeed);
    void stop();
};
#endif
