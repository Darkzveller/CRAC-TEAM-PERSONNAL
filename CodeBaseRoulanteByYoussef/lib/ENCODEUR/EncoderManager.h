// EncoderManager.h
#include <Arduino.h>

#ifndef ENCODERMANAGER_H
#define ENCODERMANAGER_H

#include <ESP32Encoder.h>

class EncoderManager
{
private:
    ESP32Encoder encoder;
    int pinA, pinB;
    float TIC_UN_TOUR;
    String nameEncodeur;
    float angle_precedent;

public:
    EncoderManager(int pinA, int pinB);

    void init(String nameEncoder, float wheel_size_mm, float TIC_ONE_TOUR);

    long getTickPosition();

    float getDistance();

    float getAngle();

    float getVitesse();

    void showTickPosition();
    
    void resetPosition();
};

#endif // ENCODERMANAGER_H
