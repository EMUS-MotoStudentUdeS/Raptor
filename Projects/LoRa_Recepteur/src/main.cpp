#include <Arduino.h>
#include <RadioLib.h>
#include <SSD1306Wire.h>
#include "../lib/encoding.h"   // pour NB_SENSOR

SX1262 radio = new Module(8, 14, 12, 13);
SSD1306Wire display(0x3c, 17, 18);

int paquetsRecus = 0;
volatile bool paquetRecu = false;

// État reconstruit — MÊME base que l'émetteur (100)
uint16_t valeurCourante[NB_SENSOR];

void IRAM_ATTR onReceive() {
    paquetRecu = true;
}

void setup() {
    Serial.begin(115200);
    delay(200);

    for (int i = 0; i < NB_SENSOR; i++) valeurCourante[i] = 100;  // baseline

    pinMode(36, OUTPUT); digitalWrite(36, LOW); delay(100);
    pinMode(21, OUTPUT); digitalWrite(21, LOW); delay(50);
    digitalWrite(21, HIGH); delay(50);

    display.init();
    display.clear();
    display.setFont(ArialMT_Plain_10);
    display.drawString(0, 0, "Recepteur");
    display.drawString(0, 15, "En attente...");
    display.display();

    int state = radio.begin(915.0);
    if (state != RADIOLIB_ERR_NONE) {
        Serial.print("Erreur LoRa: "); Serial.println(state);
        display.clear(); display.drawString(0, 0, "Erreur LoRa!"); display.display();
        while (true);
    }

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

        uint8_t buffer[1 + 2 * NB_SENSOR];
        size_t len = radio.getPacketLength();          // longueur RÉELLE, variable maintenant
        if (len > sizeof(buffer)) len = sizeof(buffer);

        int state = radio.readData(buffer, len);

        if (state == RADIOLIB_ERR_NONE && len >= 1) {
            paquetsRecus++;

            uint8_t nb = buffer[0];
            // borne : nb paires doivent tenir dans la longueur reçue
            if ((size_t)(1 + 2 * nb) <= len) {
                for (uint8_t i = 0; i < nb; i++) {
                    uint8_t idx  = buffer[1 + 2*i];
                    int8_t  delta = (int8_t)buffer[2 + 2*i];
                    if (idx < NB_SENSOR) {
                        valeurCourante[idx] += delta;   // reconstruction absolue
                    }
                }
            }

            // JSON — clés alignées sur ton mapping d'index (voir note ci-dessous)
            Serial.print("{\"tire\":");   Serial.print(valeurCourante[0]);
            Serial.print(",\"bat\":");    Serial.print(valeurCourante[1]);
            Serial.print(",\"fuel\":");   Serial.print(valeurCourante[2]);
            Serial.print(",\"speed\":");  Serial.print(valeurCourante[3]);
            Serial.print(",\"temp\":");   Serial.print(valeurCourante[4]);
            Serial.print(",\"rssi\":");   Serial.print(radio.getRSSI());
            Serial.println("}");

            display.clear();
            display.drawString(0, 0,  "Paquet #" + String(paquetsRecus));
            display.drawString(0, 15, "tire: "  + String(valeurCourante[0]));
            display.drawString(0, 27, "speed: " + String(valeurCourante[3]));
            display.drawString(0, 39, "temp: "  + String(valeurCourante[4]));
            display.display();
        } else {
            Serial.print("Erreur lecture paquet: "); Serial.println(state);
        }

        radio.startReceive();
    }
}