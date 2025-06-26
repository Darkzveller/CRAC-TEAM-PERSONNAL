#include <Arduino.h>

#ifndef _MOUVEMENT_H
#define _MOUVEMENT_H

void rotation(int consigne, int vitesse);
void ligne_droite(int consigne, int vitesse);
void x_y_theta(float coordonnee_x, float coordonnee_y, float theta_fin, int vitesse);
void asser_polaire_tick(float coordonnee_x, float coordonnee_y, float theta_cons,bool nbr_passage);

bool recalage(uint8_t direction, uint8_t type_modif, float nouvelle_valeur, uint16_t consigne_rotation);
bool toucher_objet_solid();
float determination_sens_polaire(float erreur_orient_radians);
void gestion_freinage_et_point_de_passage(float distance_de_seuil_minimal_et_changement_point_passage__mm, float seuil_minimal_autorise_permettant_de_quitter_asser_polaire_mm, float theta_cons, float seuil_minimal_permettant_de_quitter_rotation);
void reset_parametre_polaire();
#endif
