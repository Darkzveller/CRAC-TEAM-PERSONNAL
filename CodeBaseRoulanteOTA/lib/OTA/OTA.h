#include <Arduino.h>
#include <WiFi.h>
#include <ESPmDNS.h>

#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include <TelnetStream.h>

#ifndef _OTA_H
#define _OTA_H

void setupOTA();
void receptionWIFI();
#endif