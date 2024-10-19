// EncoderManager.h
#include <Arduino.h>
#include <ESP32Encoder.h>

#ifndef ENCODERMANAGER_H
#define ENCODERMANAGER_H

void setup_encodeur();
float read_encodeurdroit(uint8_t grandeur_A_Mesure);
float read_encodeurgauche(uint8_t grandeur_A_Mesure);

#endif
