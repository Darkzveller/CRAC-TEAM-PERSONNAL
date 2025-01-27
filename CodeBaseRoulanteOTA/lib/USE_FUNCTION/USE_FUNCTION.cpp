#include "Variable.h"
#include "USE_FUNCTION.h"

#include <mat.h>

int16_t fusion_octet(int octet0, int octet1)
{

    int16_t octet16 = (octet0 << 8) | octet1;
    return octet16;
}

int convert_angle_deg_to_tick(float angle)
{
    float distance_a_faire_en_mm = angle * perimetre_robot / 360;
    int consigne_roue_odo = distance_a_faire_en_mm * (TIC_PER_TOUR / (2 * M_PI * SIZE_WHEEL_DIAMETER_mm / 2.0));

    return consigne_roue_odo;
}

int convert_distance_mm_to_tick(float distance)
{

    int consigne_roue_odo = distance * (TIC_PER_TOUR / (2 * M_PI * SIZE_WHEEL_DIAMETER_mm / 2.0));
    return consigne_roue_odo;
}
void pourcentage_erreur(float val_theorique, float valeur_experimentale)
{
    float pourcent_mes = 100*((val_theorique)-valeur_experimentale)/(val_theorique);
    Serial.print(pourcent_mes);
}
