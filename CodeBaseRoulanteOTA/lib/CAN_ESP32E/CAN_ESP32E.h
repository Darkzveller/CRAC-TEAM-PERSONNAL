#ifndef _ESPCAN_H  
#define _ESPCAN_H

void setupCAN(int baudrate);
void sendCANMessage();
void readCANMessage();
bool messageCANForMe(uint16_t ID);

#endif
