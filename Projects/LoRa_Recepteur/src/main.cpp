#include <Arduino.h>
#include <RadioLib.h>
#include <SSD1306Wire.h>
#include "capteurs.h"

// Pins SX1262 du Heltec V4
SX1262 radio = new Module(8, 14, 12, 13);
SSD1306Wire display(0x3c, 17, 18);

int paquetsRecus = 0;
volatile bool paquetRecu = false;

void IRAM_ATTR onReceive() {
    paquetRecu = true;
}

void setup() {
    Serial.begin(115200);
    delay(200);

    // Alimentation écran
    pinMode(36, OUTPUT);
    digitalWrite(36, LOW);
    delay(100);

    // Reset OLED
    pinMode(21, OUTPUT);
    digitalWrite(21, LOW);
    delay(50);
    digitalWrite(21, HIGH);
    delay(50);

    // Init écran
    display.init();
    display.clear();
    display.setFont(ArialMT_Plain_10);
    display.drawString(0, 0, "Recepteur");
    display.drawString(0, 15, "En attente...");
    display.display();

    int state = radio.begin(915.0);
    if (state != RADIOLIB_ERR_NONE) {
        Serial.print("Erreur LoRa: ");
        Serial.println(state);
        display.clear();
        display.drawString(0, 0, "Erreur LoRa!");
        display.display();
        while (true);
    }

    // Config RF — DOIT être identique à l'émetteur
    radio.setSpreadingFactor(7);
    radio.setBandwidth(125.0);
    radio.setCodingRate(5);
    radio.setPreambleLength(8);

    radio.setDio1Action(onReceive);
    radio.startReceive();
    Serial.println("Recepteur pret, en ecoute 915 MHz");
}

void loop() {
    if (paquetRecu) {
        paquetRecu = false;

        uint8_t buffer[sizeof(DonneesMoto)];
        int state = radio.readData(buffer, sizeof(buffer));

        if (state == RADIOLIB_ERR_NONE) {
            paquetsRecus++;
            DonneesMoto* data = (DonneesMoto*)buffer;

            // JSON pour le serveur Python
            Serial.print("{\"temp_pneu\":");
            Serial.print(data->temp_pneu);
            Serial.print(",\"pression\":");
            Serial.print(data->pression);
            Serial.print(",\"temp_huile\":");
            Serial.print(data->temp_huile);
            Serial.print(",\"vitesse\":");
            Serial.print(data->vitesse);
            Serial.print(",\"rssi\":");
            Serial.print(radio.getRSSI());
            Serial.println("}");

            // Écran
            display.clear();
            display.drawString(0, 0,  "Paquet #" + String(paquetsRecus));
            display.drawString(0, 15, "Pneu: "  + String(data->temp_pneu)  + "C");
            display.drawString(0, 27, "Pres: "  + String(data->pression)   + "bar");
            display.drawString(0, 39, "Huile: " + String(data->temp_huile) + "C");
            display.drawString(0, 51, "Vit: "   + String(data->vitesse)    + "km/h");
            display.display();
        } else {
            Serial.print("Erreur lecture paquet: ");
            Serial.println(state);
        }

        radio.startReceive();
    }
}