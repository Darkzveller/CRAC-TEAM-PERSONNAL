#include <Arduino.h>

#ifndef _ASSERVISSEMENT_H
#define _ASSERVISSEMENT_H

// #define P
// #define PD
// #define PI
// #define PDI
// #define PID
// #define COEFF_P
// #define COEFF_PD
// #define COEFF_PI

class asservissement
{

private:
    double erreur_prec = 0, erreur_prec_gyro = 0;
    double somme_integral = 0, somme_integral_gyro = 0;

public:
    double calcul_asserv(double consigne, double observation, int resolution_pwm_bits, double coeff_P, double dt, double coeff_D, double coeff_I, double integral_limit);
    double calcul_asserv_gyro(double consigne, double observation, int resolution_pwm_bits, double coeff_P, double coeff_D, double coeff_I, double dt, double integral_limit, double saturation);
};
#endif
