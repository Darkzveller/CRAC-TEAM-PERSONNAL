#include <Arduino.h>

#ifndef _STRATEGIE_H
#define _STRATEGIE_H

void setupOTA();
void receptionWIFI(char ch);
void affichage_commande_wifi();
void SerialWIFIActivites();
void strategie_jaune_homologation();
void strategie_bleu_homologation();
void send_recalage(int x, int y, int theta);
void send_x_y_theta(int x, int y, int theta);
void send_rotation(int cmd);
void send_ligne_droite(int cmd);
#endif
