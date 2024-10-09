#include "../ID.h" // Remonte d'un niveau pour atteindre lib
#include "ServeurWebGyropode.h"
// Informations du point d'accès
const char *ssid = "Test wifi";
const char *password = "123456789";

// Déclaration du serveur web sur le port 80
AsyncWebServer server(80);

// Pin utilisé pour une LED (ou autre composant)
const int ledPin = 2; // Pin GPIO 2 pour la LED
bool isLedOn = false; // Variable pour suivre l'état de la LED

// Variables pour l'état des boutons
bool asserActif = false;
bool powerOn = false;
bool boostEnabled = false; // Variable pour l'état du boost de vitesse

// Variable prédéfinie pour la tension de batterie
float batteryVoltage = 6.5; // Exemple de valeur de tension, modifiable

// Variables pour les seuils de tension
float highVoltageThreshold = 9.0;
float mediumVoltageThreshold = 7.0;

void serveur_web_gyropode(){

 pinMode(ledPin, OUTPUT);
    digitalWrite(ledPin, LOW); // Éteindre la LED au démarrage

    // Initialisation du point d'accès Wi-Fi
    WiFi.softAP(ssid, password);

    // Afficher l'adresse IP du serveur dans le moniteur série
    Serial.println();
    Serial.println("Point d'accès Wi-Fi démarré");
    Serial.print("SSID: ");
    Serial.println(ssid);
    Serial.print("Mot de passe: ");
    Serial.println(password);

    // Attendre un court instant pour obtenir l'adresse IP
    delay(1000);

    // Afficher l'adresse IP dans le moniteur série
    Serial.print("Adresse IP du point d'accès : ");
    Serial.println(WiFi.softAPIP());

    // Page web simplifiée avec l'affichage de la tension, les boutons et le joystick
    server.on("/", HTTP_GET, [](AsyncWebServerRequest *request)
              {
        String html = R"rawliteral(
        <!DOCTYPE html>
        <html lang="fr">
        <head>
            <meta charset="UTF-8">
            <meta name="viewport" content="width=device-width, initial-scale=1.0">
            <title>IHM Gyropode</title>
            <style>
                h1 {
                    margin: 0;
                    font-size: 24px;
                    text-align: center;
                }
                body {
                    margin: 0;
                    background-color: #f0f0f0;
                    font-family: Arial, sans-serif;
                    height: 100vh;
                    overflow: hidden;
                }
                .battery {
                    position: absolute;
                    top: 20px;
                    left: 20px;
                    background-color: #fff;
                    border: 2px solid #ddd;
                    padding: 10px;
                    border-radius: 8px;
                    transition: border-color 0.3s ease;
                }
                .battery-text {
                    font-size: 18px;
                    font-weight: bold;
                }
                .controls {
                    position: absolute;
                    top: 20px;
                    right: 20px;
                    display: flex;
                    gap: 10px; /* Espacement entre les boutons */
                }
                .boost-button-container {
                    position: absolute;
                    top: 50%;
                    right: 20px;
                    transform: translateY(-50%);
                }
                button {
                    padding: 10px 20px;
                    font-size: 16px;
                    cursor: pointer;
                }
                .joystick-container {
                    position: absolute;
                    top: 100px;
                    left: 20px;
                    width: 200px;
                    height: 200px;
                }
                .joystick {
                    width: 65%;
                    height: 65%;
                    border-radius: 50%;
                    background-color: #ccc;
                    border: 3px solid #333;
                    position: absolute;
                    top: 0;
                    left: 0;
                    touch-action: none;
                }
                .stick {
                    width: 50px;
                    height: 50px;
                    border-radius: 50%;
                    background-color: #333;
                    position: absolute;
                    top: 50%;
                    left: 50%;
                    transform: translate(-50%, -50%);
                    cursor: pointer;
                }
            </style>
        </head>
        <body>
            <h1>IHM Gyropode</h1> <!-- Titre affiché sur la page web -->
            <div class="battery" id="batteryDisplay">
                <div class="battery-text">Tension: <span id="batteryVoltage">0.0V</span></div>
            </div>
            <div class="controls">
                <button id="ledButton">Asser Actif</button>
                <button id="powerButton">Power On</button>
            </div>
            <div class="boost-button-container">
                <button id="boostButton">Boost Vitesse</button>
            </div>
            <div class="joystick-container">
                <div class="joystick">
                    <div class="stick"></div>
                </div>
            </div>
            <script>
                // Variables globales pour les seuils de tension
                let highVoltageThreshold = 9.0;
                let mediumVoltageThreshold = 7.0;

                function updateBatteryDisplay(voltage) {
                    const batteryDisplay = document.getElementById('batteryDisplay');
                    if (voltage >= highVoltageThreshold) {
                        batteryDisplay.style.borderColor = 'green';
                    } else if (voltage >= mediumVoltageThreshold && voltage < highVoltageThreshold) {
                        batteryDisplay.style.borderColor = 'orange';
                    } else {
                        batteryDisplay.style.borderColor = 'red';
                    }
                }

                function fetchThresholds() {
                    fetch('/get-thresholds').then(response => response.json()).then(data => {
                        highVoltageThreshold = data.high;
                        mediumVoltageThreshold = data.medium;
                    });
                }

                fetchThresholds();

                fetch('/get-battery').then(response => response.text()).then(data => {
                    const voltage = parseFloat(data);
                    document.getElementById('batteryVoltage').innerText = voltage + 'V';
                    updateBatteryDisplay(voltage);
                });

                function updateButtonStates() {
                    fetch('/get-asser').then(response => response.text()).then(data => {
                        const asserButton = document.getElementById('ledButton');
                        if (data === 'true') {
                            asserButton.innerText = 'Asser Inactif';
                        } else {
                            asserButton.innerText = 'Asser Actif';
                        }
                    });

                    fetch('/get-power').then(response => response.text()).then(data => {
                        const powerButton = document.getElementById('powerButton');
                        if (data === 'true') {
                            powerButton.innerText = 'Power Off';
                        } else {
                            powerButton.innerText = 'Power On';
                        }
                    });

                    fetch('/get-boost').then(response => response.text()).then(data => {
                        const boostButton = document.getElementById('boostButton');
                        if (data === 'true') {
                            boostButton.innerText = 'Boost Désactivé';
                        } else {
                            boostButton.innerText = 'Boost Vitesse';
                        }
                    });
                }

                updateButtonStates();

                document.getElementById('ledButton').addEventListener('click', () => {
                    fetch('/toggle-led').then(() => {
                        updateButtonStates();
                    });
                });

                document.getElementById('powerButton').addEventListener('click', () => {
                    fetch('/toggle-power').then(() => {
                        updateButtonStates();
                    });
                });

                document.getElementById('boostButton').addEventListener('click', () => {
                    fetch('/toggle-boost').then(() => {
                        updateButtonStates();
                    });
                });

                const joystick = document.querySelector('.joystick');
                const stick = document.querySelector('.stick');
                let isDragging = false;

                joystick.addEventListener('mousedown', (e) => {
                    isDragging = true;
                    moveStick(e);
                });

                joystick.addEventListener('mouseup', () => {
                    isDragging = false;
                });

                joystick.addEventListener('mousemove', (e) => {
                    if (isDragging) {
                        moveStick(e);
                    }
                });

                joystick.addEventListener('touchstart', (e) => {
                    isDragging = true;
                    moveStick(e.touches[0]);
                });

                joystick.addEventListener('touchend', () => {
                    isDragging = false;
                });

                joystick.addEventListener('touchmove', (e) => {
                    if (isDragging) {
                        moveStick(e.touches[0]);
                    }
                });

                function moveStick(e) {
                    const rect = joystick.getBoundingClientRect();
                    const joystickX = rect.left + rect.width / 2;
                    const joystickY = rect.top + rect.height / 2;
                    const x = e.clientX || e.touches[0].clientX;
                    const y = e.clientY || e.touches[0].clientY;
                    const dx = x - joystickX;
                    const dy = y - joystickY;
                    const distance = Math.sqrt(dx * dx + dy * dy);
                    const maxDistance = rect.width / 2;
                    const clampedDistance = Math.min(distance, maxDistance);
                    const angle = Math.atan2(dy, dx);
                    const stickX = Math.cos(angle) * clampedDistance;
                    const stickY = Math.sin(angle) * clampedDistance;
                    stick.style.transform = `translate(-50%, -50%) translate(${stickX}px, ${stickY}px)`;
                    sendJoystickData(stickX, stickY);
                }

                function sendJoystickData(x, y) {
                    fetch(`/joystick?x=${x}&y=${y}`)
                        .then(response => response.text())
                        .then(data => console.log(data))
                        .catch(error => console.error('Erreur:', error));
                }
            </script>
        </body>
        </html>
        )rawliteral";
        request->send(200, "text/html", html); });

    // API pour récupérer les seuils de tension
    server.on("/get-thresholds", HTTP_GET, [](AsyncWebServerRequest *request)
              {
        String response = "{\"high\":" + String(highVoltageThreshold) + ",\"medium\":" + String(mediumVoltageThreshold) + "}";
        request->send(200, "application/json", response); });

    // API pour récupérer l'état de la LED
    server.on("/get-asser", HTTP_GET, [](AsyncWebServerRequest *request)
              {
        String state = asserActif ? "true" : "false";
        request->send(200, "text/plain", state); });

    // API pour basculer l'état de la LED
    server.on("/toggle-led", HTTP_GET, [](AsyncWebServerRequest *request)
              {
        asserActif = !asserActif; // Inverser l'état de la LED
        Serial.print("Bouton Asser pressé. Nouvel état : ");
        Serial.println(asserActif ? "Actif" : "Inactif");
        int ledState = digitalRead(ledPin);
        digitalWrite(ledPin, !ledState); // Changer l'état de la LED
        request->send(200, "text/plain", asserActif ? "Actif" : "Inactif"); });

    // API pour récupérer l'état de l'alimentation
    server.on("/get-power", HTTP_GET, [](AsyncWebServerRequest *request)
              {
        String state = powerOn ? "true" : "false";
        request->send(200, "text/plain", state); });

    // API pour basculer l'état de l'alimentation
    server.on("/toggle-power", HTTP_GET, [](AsyncWebServerRequest *request)
              {
        powerOn = !powerOn; // Inverser l'état de l'alimentation
        Serial.print("Bouton Power pressé. Nouvel état : ");
        Serial.println(powerOn ? "On" : "Off");
        request->send(200, "text/plain", powerOn ? "On" : "Off"); });

    // API pour récupérer l'état du boost de vitesse
    server.on("/get-boost", HTTP_GET, [](AsyncWebServerRequest *request)
              {
        String state = boostEnabled ? "true" : "false";
        request->send(200, "text/plain", state); });

    // API pour activer/désactiver le boost de vitesse
    server.on("/toggle-boost", HTTP_GET, [](AsyncWebServerRequest *request)
              {
        boostEnabled = !boostEnabled; // Inverser l'état du boost de vitesse
        Serial.print("Bouton Boost Vitesse pressé. Nouvel état : ");
        Serial.println(boostEnabled ? "Activé" : "Désactivé");
        request->send(200, "text/plain", boostEnabled ? "Activé" : "Désactivé"); });

    // API pour obtenir la tension de la batterie
    server.on("/get-battery", HTTP_GET, [](AsyncWebServerRequest *request)
              {
        String voltage = String(batteryVoltage);
        request->send(200, "text/plain", voltage); });

    // API pour recevoir les données du joystick
    server.on("/joystick", HTTP_GET, [](AsyncWebServerRequest *request)
              {
        String x = request->getParam("x")->value();
        String y = request->getParam("y")->value();
        Serial.print("Joystick X: ");
        Serial.print(x);
        Serial.print(" Y: ");
        Serial.println(y);
        request->send(200, "text/plain", "Joystick Data Received"); });

    // Lancer le serveur
    server.begin();

}