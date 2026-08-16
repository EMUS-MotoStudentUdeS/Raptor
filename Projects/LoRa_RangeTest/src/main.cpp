#include <Arduino.h>
#include <RadioLib.h>
#include <SSD1306Wire.h>

// ============================================================
//  PROJET ISOLE DE TEST DE PORTEE LORA
//  Aucune dependance CAN/capteurs/encodage : juste TX <-> RX avec
//  un compteur 4 octets, pour tester la portee/qualite du lien
//  radio (RSSI, SNR, paquets perdus) sans le bruit du reste du
//  firmware. Reprend tous les fixes RF valides sur LoRa_Comm /
//  LoRa_Recepteur (DIO2, TCXO, OCP, front-end GC1109, RX boost).
//
//  1 = ce flash est l'EMETTEUR. 0 = ce flash est le RECEPTEUR.
//  Flasher CHAQUE carte separement avec la bonne valeur.
// ============================================================
#define LORA_ROLE_TX 0

// ============================================================
//  MATERIEL - Heltec WiFi LoRa 32 V4 (ESP32-S3 + SX1262)
// ============================================================
SX1262 radio = new Module(8, 14, 12, 13);   // NSS, DIO1, RESET, BUSY
SSD1306Wire display(0x3c, 17, 18);          // SDA, SCL

// Front-end RF externe (ampli GC1109) : sans ca la carte reste en
// bypass basse puissance. Voir LoRa_Comm/main.cpp pour le detail.
#define FEM_VFEM 7
#define FEM_CSD  2
#define FEM_CPS  5

const unsigned long INTERVALLE_ENVOI = 200;   // ms
unsigned long dernierEnvoi = 0;
uint32_t compteur = 0;

// Puissance demandee au SX1262 (pas une mesure reelle : il n'y a pas de
// capteur de puissance RF sur cette carte, et RadioLib n'expose pas de
// lecture de la puissance reellement en sortie. Cette valeur ne reflete
// donc pas le gain ajoute en aval par l'ampli GC1109).
const int8_t TX_POWER_DBM = 17;   // V4.3.1 : 22dBm sature le GC1109 en entree
const float  TX_POWER_MW  = powf(10.0f, TX_POWER_DBM / 10.0f);   // dBm -> mW

#if !LORA_ROLE_TX
volatile bool paquetRecu = false;
bool premierPaquet = true;
uint32_t dernierCompteurRecu = 0;
uint32_t paquetsPerdus = 0;
uint32_t paquetsRecus = 0;

void IRAM_ATTR onReceive()
{
    paquetRecu = true;
}
#endif


void setup()
{
    Serial.begin(115200);
    delay(2000);

    // --- Alimentation + reset OLED ---
    pinMode(36, OUTPUT); digitalWrite(36, LOW);  delay(100);
    pinMode(21, OUTPUT); digitalWrite(21, LOW);  delay(50);
    digitalWrite(21, HIGH); delay(50);

    display.init();
    display.clear();
    display.setFont(ArialMT_Plain_10);
    display.drawString(0, 0, "Init...");
    display.display();

    // --- Front-end RF (GC1109) AVANT la radio ---
    pinMode(FEM_VFEM, OUTPUT); digitalWrite(FEM_VFEM, HIGH);
    pinMode(FEM_CSD,  OUTPUT); digitalWrite(FEM_CSD,  HIGH);
    pinMode(FEM_CPS,  OUTPUT); digitalWrite(FEM_CPS,  HIGH);
    delay(10);

    // --- Init LoRa ---
    int state = radio.begin(915.0);
    if (state != RADIOLIB_ERR_NONE)
    {
        Serial.print("Erreur LoRa: "); Serial.println(state);
        display.clear(); display.drawString(0, 0, "Erreur LoRa!"); display.display();
        while (true);
    }

    if (radio.setDio2AsRfSwitch(true) != RADIOLIB_ERR_NONE)
    {
        Serial.println("Attention : setDio2AsRfSwitch a echoue");
    }
    if (radio.setTCXO(1.8) != RADIOLIB_ERR_NONE)
    {
        Serial.println("Attention : setTCXO(1.8) a echoue");
    }

    // --- Parametres RF : DOIVENT etre identiques des deux cotes ---
    radio.setSpreadingFactor(8);
    radio.setBandwidth(125.0);
    radio.setCodingRate(5);
    radio.setPreambleLength(8);
    radio.setSyncWord(0x12);
    radio.setCRC(true);

    if (radio.setCurrentLimit(140) != RADIOLIB_ERR_NONE)
    {
        Serial.println("Attention : setCurrentLimit(140) a echoue");
    }
    if (radio.setOutputPower(TX_POWER_DBM) != RADIOLIB_ERR_NONE)
    {
        Serial.println("Attention : setOutputPower a echoue");
    }

#if LORA_ROLE_TX
    Serial.println(String("Puissance demandee au SX1262 : ") + TX_POWER_DBM
                   + "dBm (" + String(TX_POWER_MW, 1) + "mW) - hors gain GC1109");

    display.clear();
    display.drawString(0, 0, "TEST PORTEE");
    display.drawString(0, 15, "EMETTEUR");
    display.drawString(0, 30, String(TX_POWER_DBM) + "dBm / " + String(TX_POWER_MW, 0) + "mW");
    display.display();
    Serial.println("Emetteur pret.");
#else
    if (radio.setRxBoostedGainMode(true) != RADIOLIB_ERR_NONE)
    {
        Serial.println("Attention : setRxBoostedGainMode(true) a echoue");
    }
    radio.setDio1Action(onReceive);
    radio.startReceive();

    display.clear();
    display.drawString(0, 0, "TEST PORTEE");
    display.drawString(0, 15, "RECEPTEUR");
    display.drawString(0, 30, "En attente...");
    display.display();
    Serial.println("Recepteur pret, en ecoute 915 MHz.");
#endif
}


void loop()
{
#if LORA_ROLE_TX
    unsigned long maintenant = millis();
    if (maintenant - dernierEnvoi < INTERVALLE_ENVOI)
    {
        return;
    }
    dernierEnvoi = maintenant;

    uint8_t paquet[4] = {
        (uint8_t)(compteur >> 0),
        (uint8_t)(compteur >> 8),
        (uint8_t)(compteur >> 16),
        (uint8_t)(compteur >> 24),
    };

    int state = radio.transmit(paquet, sizeof(paquet));
    if (state == RADIOLIB_ERR_NONE)
    {
        Serial.println(String("Envoye #") + compteur);
    }
    else
    {
        Serial.println(String("Erreur envoi : ") + state);
    }

    display.clear();
    display.drawString(0, 0,  "TEST PORTEE - TX");
    display.drawString(0, 15, "Envoyes: " + String(compteur));
    display.drawString(0, 30, String(TX_POWER_DBM) + "dBm / " + String(TX_POWER_MW, 0) + "mW");
    display.display();

    compteur++;
#else
    if (!paquetRecu)
    {
        return;
    }
    paquetRecu = false;

    uint8_t buffer[4];
    size_t len = radio.getPacketLength();
    if (len > sizeof(buffer)) len = sizeof(buffer);

    int state = radio.readData(buffer, len);
    if (state == RADIOLIB_ERR_NONE && len == 4)
    {
        paquetsRecus++;

        uint32_t recu = (uint32_t)buffer[0]
                       | ((uint32_t)buffer[1] << 8)
                       | ((uint32_t)buffer[2] << 16)
                       | ((uint32_t)buffer[3] << 24);

        if (premierPaquet)
        {
            premierPaquet = false;
        }
        else
        {
            uint32_t attendu = dernierCompteurRecu + 1;
            if (recu > attendu)
            {
                paquetsPerdus += (recu - attendu);
            }
        }
        dernierCompteurRecu = recu;

        float rssi = radio.getRSSI();
        float snr  = radio.getSNR();

        Serial.println(String("Recu #") + recu
                       + "  RSSI=" + rssi + "dBm"
                       + "  SNR=" + snr + "dB"
                       + "  perdus=" + paquetsPerdus);

        display.clear();
        display.drawString(0, 0,  "TEST PORTEE - RX");
        display.drawString(0, 15, "Paquet #" + String(recu));
        display.drawString(0, 27, "RSSI: " + String(rssi, 1) + " dBm");
        display.drawString(0, 39, "SNR: " + String(snr, 1) + " dB");
        display.drawString(0, 51, "Perdus: " + String(paquetsPerdus));
        display.display();
    }
    else
    {
        Serial.print("Erreur lecture paquet: "); Serial.println(state);
    }

    radio.startReceive();
#endif
}
