#ifndef _ESPCAN_H
#define _ESPCAN_H

void setupCAN(int baudrate);
void sendCANMessage(int id, int ext, int rtr, int length, int data0, int data1, int data2, int data3, int data4, int data5, int data6, int data7);
void readCANMessage();
void verificateur_acquitement(int id, int data);
#endif
