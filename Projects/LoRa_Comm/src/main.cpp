#include <Arduino.h>
#include <RadioLib.h>
#include <SSD1306Wire.h>
#include "capteurs.h"

// Pins SX1262 du Heltec V4
SX1262 radio = new Module(8, 14, 12, 13);
SSD1306Wire display(0x3c, 17, 18);

int compteur = 0;
//bool canOK = false;

unsigned long dernierEnvoi = 0;
unsigned long dernierDiag  = 0;



//===============================================================================
//DESH compression données
//===============================================================================
#include "../lib/encoding.h"

DeltaEncoder encoder;
int sensors[10] = {100,100,100,100,100,100,100,100,100,100};
uint8_t output_index[NB_SENSOR] = {0};
int output_diff[NB_SENSOR] = {0};
uint16_t nb_donnees = 0;
int valeur_arbitraire = 100;
//===============================================================================

void print_Lora(int x, int y, const char* text, bool clearFirst)
{
    static int nb_lines = 0;

    if (clearFirst) 
    {
        display.clear();
        nb_lines = 0;
    }
    if (nb_lines >= 4) 
    {
        delay(1000);
        display.clear();
        Serial.println("Affichage réinitialisé-------------------------------------");
        nb_lines = 0;
    }
    display.drawString(x, y, text);
    display.display();

    nb_lines++;
}

void setup() {
//===============================================================================
//DESH compression données
//===============================================================================



    Serial.begin(115200);
    delay(5000);

    // Activer alimentation écran
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
    display.drawString(0, 0, "Initialisation...");
    display.display();

    Serial.print("Initialisation LoRa... ");

    int state = radio.begin(915.0);
    if (state == RADIOLIB_ERR_NONE) {
        Serial.println("OK!");
    } 
    else {
        Serial.print("Erreur: ");
        Serial.println(state);
        display.clear();
        display.drawString(0, 0, "Erreur LoRa!");
        display.display();
        while (true);
    }

    radio.setSpreadingFactor(7);
    radio.setBandwidth(125.0);
    radio.setCodingRate(5);
    radio.setOutputPower(22);
    radio.setPreambleLength(8);
    Serial.println("Config RF appliquée!");

    // Init CAN
//    canOK = initCAN();

    // État initial sur l'écran    
    print_Lora(0, 0, "Emetteur", true);
    print_Lora(0, 15, "LoRa OK", false);





//==========================================================================================================================================
//DESH compression données
//==========================================================================================================================================
encoder.init(sensors);
encoder.setName(0, "tire");
encoder.setName(1, "bat");
encoder.setName(2, "fuel");
encoder.setName(3, "speed");
encoder.setName(4, "temp");
for (int i = 0; i < NB_SENSOR; i++) {
    encoder.setThreshold(i, i + 1); // seuil de détection pour chaque capteur
    encoder.setPriority(i, 0);
}





}

void loop() {
    // 1) Vider les trames CAN en attente (rapide, non-bloquant)
//    if (canOK) {
//        majDonneesCAN();
//    }

    // 2) Transmettre en LoRa aux 200 ms
    if (millis() - dernierEnvoi >= 200) {
        dernierEnvoi = millis();
//        DonneesMoto data = lireCapteurs();
//        int state = radio.transmit((uint8_t*)&data, sizeof(data));

        // if (state == RADIOLIB_ERR_NONE) {
        //     compteur++;
        //     display.clear();
        //     display.drawString(0, 0, "Emetteur");
        //     display.drawString(0, 15, canOK ? "CAN OK" : "CAN ERREUR!");
        //     display.drawString(0, 30, "Paquet #" + String(compteur));
        //     display.display();
        // } else {
            // Serial.print("Erreur envoi: ");
            // Serial.println(state);
            // display.clear();
            // display.drawString(0, 0, "Emetteur");
            // display.drawString(0, 15, "Erreur envoi!");
            // display.display();
       // }
    }

    // 3) Diagnostic CAN aux 2 secondes
    // if (canOK && millis() - dernierDiag >= 2000) {
    //     dernierDiag = millis();
    //     diagCAN();
    //}


//===============================================================================
//DESH compression données
//===============================================================================

valeur_arbitraire += 2;
for (int i = 0; i < NB_SENSOR; i++) {
    sensors[i] = valeur_arbitraire;
}

nb_donnees = encoder.sorting(sensors, output_index, output_diff);
String text = String("Nb capteurs: ") + nb_donnees;
print_Lora(0, 0, text.c_str(), true);
Serial.println(String("\nOutput: ") + nb_donnees);
Serial.println("-------------------------------");
for (int i = 0; i < nb_donnees; i++) {
    String line = String(encoder.getConfigByIndex(output_index[i]).name) + ", " + output_diff[i] + ", " + encoder.getConfigByIndex(output_index[i]).last_sent;
    Serial.println(line);
    print_Lora(0, ((i+1)%4) * 15, line.c_str(), false);
}


delay(1000);
}
