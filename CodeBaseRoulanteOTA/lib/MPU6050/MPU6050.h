#include <Arduino.h>
#include <math.h>
#include <Wire.h>
#include <String.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#ifndef MPU6050_H
#define MPU6050_H


class MPU6050
{
private:
    Adafruit_MPU6050 mpu;
    sensors_event_t a, g, temp;

   
public:
void init();
double getPosAngulaireRad();
 void getPosAngulaireDegres();

};

#endif // ENCODERMANAGER_H
