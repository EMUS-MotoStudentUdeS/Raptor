#include "capteurs.h"
#include <SPI.h>
#include <Arduino.h>

#define CS_PIN   3
#define SCK_PIN  4
#define MOSI_PIN 5
#define MISO_PIN 6

#define MCP_RESET    0xC0
#define MCP_READ     0x03
#define MCP_WRITE    0x02
#define REG_CANSTAT  0x0E
#define REG_CANCTRL  0x0F
#define REG_CNF1     0x2A
#define REG_CNF2     0x29
#define REG_CNF3     0x28
#define REG_CANINTF  0x2C
#define REG_REC      0x1D
#define REG_RXB0CTRL 0x60
#define REG_RXB0SIDH 0x61
#define REG_RXB0SIDL 0x62
#define REG_RXB0DLC  0x65
#define REG_RXB0D0   0x66

SPIClass spiCAN(HSPI);

// Données courantes, mises à jour au fil des trames CAN reçues
static DonneesMoto donneesCourantes = {0.0f, 0.0f, 0.0f, 0.0f};

// ---------- SPI bas niveau ----------

static uint8_t spi_read(uint8_t reg) {
    spiCAN.beginTransaction(SPISettings(125000, MSBFIRST, SPI_MODE0));
    digitalWrite(CS_PIN, LOW);
    spiCAN.transfer(MCP_READ);
    spiCAN.transfer(reg);
    uint8_t val = spiCAN.transfer(0x00);
    digitalWrite(CS_PIN, HIGH);
    spiCAN.endTransaction();
    return val;
}

static void spi_write(uint8_t reg, uint8_t val) {
    spiCAN.beginTransaction(SPISettings(125000, MSBFIRST, SPI_MODE0));
    digitalWrite(CS_PIN, LOW);
    spiCAN.transfer(MCP_WRITE);
    spiCAN.transfer(reg);
    spiCAN.transfer(val);
    digitalWrite(CS_PIN, HIGH);
    spiCAN.endTransaction();
}

// ---------- Initialisation ----------

bool initCAN() {
    pinMode(CS_PIN, OUTPUT);
    digitalWrite(CS_PIN, HIGH);
    spiCAN.begin(SCK_PIN, MISO_PIN, MOSI_PIN, -1);
    delay(10);

    // Reset
    spiCAN.beginTransaction(SPISettings(125000, MSBFIRST, SPI_MODE0));
    digitalWrite(CS_PIN, LOW);
    spiCAN.transfer(MCP_RESET);
    digitalWrite(CS_PIN, HIGH);
    spiCAN.endTransaction();
    delay(200);

    // Vérif SPI : après reset, mode Config = 0x80
    uint8_t canstat = spi_read(REG_CANSTAT) & 0xE0;
    Serial.print("CANSTAT apres reset: 0x");
    Serial.println(canstat, HEX);
    if (canstat != 0x80) {
        Serial.println(">>> ECHEC ETAPE A : SPI ne repond pas");
        return false;
    }
    Serial.println("ETAPE A OK : SPI fonctionne");

    // Bit timing 500 kbps @ crystal 8 MHz
    spi_write(REG_CNF1, 0x01); // <- 0x00 devient 0x01 (BRP=1)
    spi_write(REG_CNF2, 0x90);
    spi_write(REG_CNF3, 0x02);

    // Vérif écriture : relire CNF2
    if (spi_read(REG_CNF2) != 0x90) {
        Serial.println(">>> ECHEC ETAPE B : ecriture registre echoue");
        return false;
    }
    Serial.println("ETAPE B OK : registres configurables");

    // Accepter toutes les trames (pas de filtre) + rollover
    spi_write(REG_RXB0CTRL, 0x64);

    // Mode Normal
    spi_write(REG_CANCTRL, 0x00);
    delay(10);
    uint8_t mode = spi_read(REG_CANSTAT) & 0xE0;
    if (mode != 0x00) {
        Serial.print(">>> ECHEC ETAPE C : refus mode Normal, CANSTAT=0x");
        Serial.println(mode, HEX);
        return false;
    }
    Serial.println("ETAPE C OK : mode Normal, en ecoute!");
    return true;
}

// ---------- Réception ----------

bool lireTrameCAN(uint16_t* id, uint8_t* data, uint8_t* len) {
    if (!(spi_read(REG_CANINTF) & 0x01)) return false;

    uint8_t sidh = spi_read(REG_RXB0SIDH);
    uint8_t sidl = spi_read(REG_RXB0SIDL);
    *id = ((uint16_t)sidh << 3) | (sidl >> 5);
    *len = spi_read(REG_RXB0DLC) & 0x0F;
    if (*len > 8) *len = 8;
    for (uint8_t i = 0; i < *len; i++) data[i] = spi_read(REG_RXB0D0 + i);

    spi_write(REG_CANINTF, 0x00);   // libérer le buffer
    return true;
}

// Vide toutes les trames en attente et met à jour donneesCourantes.
// À appeler souvent dans loop().
void majDonneesCAN() {
    uint16_t id;
    uint8_t data[8];
    uint8_t len;

    while (lireTrameCAN(&id, data, &len)) {
        // Debug : afficher chaque trame reçue
        Serial.print("TRAME ID=0x");
        Serial.print(id, HEX);
        Serial.print(" len=");
        Serial.print(len);
        Serial.print(" data:");
        for (uint8_t i = 0; i < len; i++) {
            Serial.print(" ");
            if (data[i] < 0x10) Serial.print("0");
            Serial.print(data[i], HEX);
        }
        Serial.println();

        // Mapping ID -> DonneesMoto
        // >>> ID et formules BIDONS : à remplacer quand on connaîtra
        //     les vrais ID de ton AiM (on les verra défiler en debug)
        switch (id) {
            // case 0x100:  // exemple : temp pneu sur 2 octets, 0.1 C/bit
            //     donneesCourantes.temp_pneu =
            //         ((uint16_t)(data[0] << 8) | data[1]) * 0.1f;
            //     break;
            // case 0x101:
            //     donneesCourantes.vitesse =
            //         ((uint16_t)(data[0] << 8) | data[1]) * 0.1f;
            //     break;
            default:
                break;
        }
    }
}

DonneesMoto lireCapteurs() {
    return donneesCourantes;
}

// ---------- Diagnostic ----------

// Affiche le compteur d'erreurs de réception.
// REC=0 constant -> aucune activité vue sur le bus
// REC qui grimpe -> le bus parle mais mauvais bitrate
void diagCAN() {
    Serial.print("[diag] REC=");
    Serial.println(spi_read(REG_REC));
}