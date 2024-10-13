#include <Arduino.h>
#include "OTA.h"

int i = 0;

void setup()
{
    setupOTA();
    Serial.begin(115200);
    Serial.println("Bonjour Mon chers OTA");
}

void loop()
{
    ArduinoOTA.handle();
    Serial.println(i++);
    delay(100);
}