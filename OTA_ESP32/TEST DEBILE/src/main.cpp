

#include "OTA.h"

void setup()
{
    Serial.begin(115200);
    Serial.println("Booting with Telnet");

    setupOTA();
    while (!SerialWIFI.available())
    {
        delay(500);
        Serial.println("Aucun client connecté, en attente...");
    }
    SerialWIFI.println("Coucou");

    // Your setup code
}

void loop()
{
    static int i = 0;

    SerialWIFI.println(i++);
    delay(500);
}