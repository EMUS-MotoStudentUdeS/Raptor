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
#define REG_RXB0EID8 0x63
#define REG_RXB0EID0 0x64
#define REG_RXB0DLC  0x65
#define REG_RXB0D0   0x66

#define SIDL_IDE_BIT 0x08   // RXB0SIDL bit3 : 1 = trame etendue (29 bits), 0 = standard (11 bits)

SPIClass spiCAN(HSPI);

// Données courantes, mises à jour au fil des trames CAN reçues
static DonneesMoto donneesCourantes = {0.0f, 0.0f, 0.0f, 0.0f};

// Valeurs brutes decodees depuis le DBC (indices IDX_BATTERY_MAX_TEMP et
// suivants). Les indices legacy (0-9) ne sont jamais ecrits ici.
static int dbcRaw[NB_SENSOR] = {0};

// Noms lisibles, un par index de SensorIndex, DANS LE MEME ORDRE que l'enum
// (voir capteurs.h). Source unique utilisee par l'emetteur ET le recepteur.
const char* const kSensorNames[NB_SENSOR] = {
    "tire", "bat", "fuel", "speed", "temp",
    "res_5", "res_6", "res_7", "res_8", "res_9",

    "batt_max_temp", "batt_max_volt", "batt_min_volt",

    "komodo_hb8", "komodo_b1", "komodo_b2", "komodo_b3",
    "komodo_b4", "komodo_b5", "komodo_b6",

    "throttle_req", "torque", "key_on_time", "odometer_lo", "odometer_hi",

    "fault_code", "fault_level",

    "iq_ref", "i_motor", "i_bat", "v_bat",

    "key_sw_volt", "motor_temp", "inverter_temp", "speed_ref", "speed_measure",

    "bender_earth", "bender_imd", "bender_spd_bad", "bender_spd_ok",
    "bender_uv", "bender_happy", "bender_short",
    "cluster_chg", "cluster_unsafe", "cluster_eng", "cluster_safe",
    "batt_overv", "bender_fault", "batt_disc", "batt_underv",
    "batt_overtemp", "e_stop",

    "b_button", "fwd_switch", "mode3", "mode2", "mode1",

    "current_sensor", "flow_drive", "flow_motor",

    "chg_heartbeat",

    "chg_maxv_hi", "chg_maxi_hi", "chg_control", "chg_maxv_lo", "chg_maxi_lo",

    "chg_hw_fail", "chg_overtemp", "chg_wrong_vin", "chg_no_batt",
    "chg_comm_err", "chg_iout_hi", "chg_iout_lo", "chg_vout_hi", "chg_vout_lo",
};

// ---------- Decodage generique de signaux DBC (format Intel/@1, little-endian) ----------

// Assemble les 8 octets de la trame en un mot 64 bits little-endian (data[0] =
// octet de poids faible), puis extrait 'length' bits a partir de 'startBit'.
// C'est exactement la convention "Intel" utilisee par toutes les SG_ de ce DBC.
static uint64_t extractBitsLE(const uint8_t data[8], uint8_t startBit, uint8_t length) {
    uint64_t word = 0;
    for (int8_t i = 7; i >= 0; i--) {
        word = (word << 8) | data[i];
    }
    uint64_t mask = (length >= 64) ? ~0ULL : ((1ULL << length) - 1ULL);
    return (word >> startBit) & mask;
}

// Etend le signe d'une valeur brute de 'length' bits (complement a 2).
// Ne pas utiliser sur un signal de 1 bit : un booleen DBC marque "signe" (-)
// n'a pas de sens negatif, on garde alors le bit brut tel quel (voir appels).
static int32_t signExtend(uint64_t raw, uint8_t length) {
    if (length >= 32) return (int32_t)raw;
    uint32_t signBit = 1UL << (length - 1);
    return (int32_t)(((uint32_t)raw ^ signBit) - signBit);
}

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

bool lireTrameCAN(uint32_t* id, uint8_t* data, uint8_t* len) {
    if (!(spi_read(REG_CANINTF) & 0x01)) return false;

    uint8_t sidh = spi_read(REG_RXB0SIDH);
    uint8_t sidl = spi_read(REG_RXB0SIDL);
    uint32_t sid = ((uint32_t)sidh << 3) | (sidl >> 5);   // 11 bits (standard OU partie haute d'une trame etendue)

    if (sidl & SIDL_IDE_BIT) {
        // Trame etendue (29 bits) : SID (11 bits) + EID (18 bits, reparti sur
        // SIDL[1:0] + EID8 + EID0).
        uint8_t eid8 = spi_read(REG_RXB0EID8);
        uint8_t eid0 = spi_read(REG_RXB0EID0);
        uint32_t eid = ((uint32_t)(sidl & 0x03) << 16) | ((uint32_t)eid8 << 8) | eid0;
        *id = (sid << 18) | eid;
    } else {
        // Trame standard (11 bits)
        *id = sid;
    }

    *len = spi_read(REG_RXB0DLC) & 0x0F;
    if (*len > 8) *len = 8;
    for (uint8_t i = 0; i < *len; i++) data[i] = spi_read(REG_RXB0D0 + i);

    spi_write(REG_CANINTF, 0x00);   // libérer le buffer
    return true;
}

// Vide toutes les trames en attente et met à jour donneesCourantes.
// À appeler souvent dans loop().
void majDonneesCAN() {
    uint32_t id;
    uint8_t data[8] = {0};   // zero-init : les signaux DBC supposent une trame de 8 octets complete
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

        // ====================================================
        //  Mapping ID -> DonneesMoto (test BusMaster)
        //  Chaque trame porte sa valeur sur le 1er octet (data[0]),
        //  soit une valeur entiere de 0 a 255. On la stocke en float
        //  dans donneesCourantes ; l'emetteur la recoupera ensuite
        //  en int pour l'encodage delta.
        // ====================================================
        switch (id) {
            case 0x417:
                donneesCourantes.temp_pneu = (float)data[0];
                break;
            case 0x427:
                donneesCourantes.pression = (float)data[0];
                break;
            case 0x437:
                donneesCourantes.temp_huile = (float)data[0];
                break;
            case 0x457:
                donneesCourantes.vitesse = (float)data[0];
                break;

            // ====================================================
            //  Signaux issus du DBC : plusieurs capteurs par trame,
            //  chacun a son propre (start_bit, length, signe) tel
            //  que defini par les SG_ du fichier. Format Intel (@1)
            //  partout -> extractBitsLE gere l'assemblage little-endian.
            //  Valeurs stockees BRUTES (scale/offset a appliquer plus
            //  tard, cote affichage).
            // ====================================================

            case 6:   // ecu_battery
                dbcRaw[IDX_BATTERY_MAX_TEMP]    = (int)extractBitsLE(data, 0, 16);
                dbcRaw[IDX_BATTERY_MAX_VOLTAGE] = (int)extractBitsLE(data, 16, 16);
                dbcRaw[IDX_BATTERY_MIN_VOLTAGE] = (int)extractBitsLE(data, 32, 16);
                break;

            case 1538:   // komodo
                dbcRaw[IDX_KOMODO_HEARTBEAT_8] = (int)extractBitsLE(data, 8, 8);
                dbcRaw[IDX_KOMODO_B1] = (int)extractBitsLE(data, 0, 1);
                dbcRaw[IDX_KOMODO_B2] = (int)extractBitsLE(data, 1, 1);
                dbcRaw[IDX_KOMODO_B3] = (int)extractBitsLE(data, 2, 1);
                dbcRaw[IDX_KOMODO_B4] = (int)extractBitsLE(data, 3, 1);
                dbcRaw[IDX_KOMODO_B5] = (int)extractBitsLE(data, 4, 1);
                dbcRaw[IDX_KOMODO_B6] = (int)extractBitsLE(data, 5, 1);
                break;

            case 84:   // drive_electric_2
                dbcRaw[IDX_THROTTLE_REQUEST]    = signExtend(extractBitsLE(data, 0, 8), 8);
                dbcRaw[IDX_TORQUE]               = signExtend(extractBitsLE(data, 8, 8), 8);
                dbcRaw[IDX_SYSTEM_KEY_ON_TIME]   = (int)extractBitsLE(data, 16, 8);
                dbcRaw[IDX_ODOMETER_LOW]         = (int)extractBitsLE(data, 32, 16);
                dbcRaw[IDX_ODOMETER_HIGH]        = (int)extractBitsLE(data, 48, 16);
                break;

            case 86:   // drive_status
                dbcRaw[IDX_FAULT_CODE]  = (int)extractBitsLE(data, 0, 8);
                dbcRaw[IDX_FAULT_LEVEL] = (int)extractBitsLE(data, 8, 8);
                break;

            case 82:   // drive_electric_1
                dbcRaw[IDX_IQ_REF]   = signExtend(extractBitsLE(data, 16, 16), 16);
                dbcRaw[IDX_I_MOTOR]  = signExtend(extractBitsLE(data, 32, 16), 16);
                dbcRaw[IDX_I_BAT]    = signExtend(extractBitsLE(data, 48, 16), 16);
                dbcRaw[IDX_V_BAT]    = signExtend(extractBitsLE(data, 0, 16), 16);
                break;

            case 81:   // drive_speed_temp
                dbcRaw[IDX_KEY_SWITCH_VOLT] = signExtend(extractBitsLE(data, 48, 16), 16);
                dbcRaw[IDX_MOTOR_TEMP]      = (int)extractBitsLE(data, 0, 8);
                dbcRaw[IDX_INVERTER_TEMP]   = (int)extractBitsLE(data, 8, 8);
                dbcRaw[IDX_SPEED_REF]       = signExtend(extractBitsLE(data, 16, 16), 16);
                dbcRaw[IDX_SPEED_MEASURE]   = signExtend(extractBitsLE(data, 32, 16), 16);
                break;

            case 3:   // ecu_fault_list (tous des drapeaux 1 bit -> bit brut, pas de sign-extend)
                dbcRaw[IDX_BENDER_EARTH_FAULT]           = (int)extractBitsLE(data, 22, 1);
                dbcRaw[IDX_BENDER_IMD_FAULT]              = (int)extractBitsLE(data, 21, 1);
                dbcRaw[IDX_BENDER_SPEED_START_BAD]        = (int)extractBitsLE(data, 20, 1);
                dbcRaw[IDX_BENDER_SPEED_START_GOOD]       = (int)extractBitsLE(data, 19, 1);
                dbcRaw[IDX_BENDER_UV_FAULT]                = (int)extractBitsLE(data, 18, 1);
                dbcRaw[IDX_BENDER_HAPPY]                   = (int)extractBitsLE(data, 17, 1);
                dbcRaw[IDX_BENDER_SHORT_CIRCUIT_FAULT]     = (int)extractBitsLE(data, 16, 1);
                dbcRaw[IDX_CLUSTER_CHARGING]               = (int)extractBitsLE(data, 11, 1);
                dbcRaw[IDX_CLUSTER_NOT_SAFE]                = (int)extractBitsLE(data, 10, 1);
                dbcRaw[IDX_CLUSTER_ENGAGED]                 = (int)extractBitsLE(data, 9, 1);
                dbcRaw[IDX_CLUSTER_SAFE]                    = (int)extractBitsLE(data, 8, 1);
                dbcRaw[IDX_BATTERY_OVER_VOLTAGE_FAULT]      = (int)extractBitsLE(data, 0, 1);
                dbcRaw[IDX_BENDER_FAULT]                    = (int)extractBitsLE(data, 4, 1);
                dbcRaw[IDX_BATTERY_DISCONNECTION_FAULT]     = (int)extractBitsLE(data, 3, 1);
                dbcRaw[IDX_BATTERY_UNDER_VOLTAGE_FAULT]     = (int)extractBitsLE(data, 2, 1);
                dbcRaw[IDX_BATTERY_OVER_TEMP_FAULT]         = (int)extractBitsLE(data, 1, 1);
                dbcRaw[IDX_E_STOP]                          = (int)extractBitsLE(data, 5, 1);
                break;

            case 9:   // ecu_modes (tous des drapeaux 1 bit)
                dbcRaw[IDX_B_BUTTON]       = (int)extractBitsLE(data, 4, 1);
                dbcRaw[IDX_FORWARD_SWITCH] = (int)extractBitsLE(data, 3, 1);
                dbcRaw[IDX_MODE3]          = (int)extractBitsLE(data, 2, 1);
                dbcRaw[IDX_MODE2]          = (int)extractBitsLE(data, 1, 1);
                dbcRaw[IDX_MODE1]          = (int)extractBitsLE(data, 0, 1);
                break;

            case 4:   // ecu_sensors
                dbcRaw[IDX_CURRENT_SENSOR]     = (int)extractBitsLE(data, 32, 32);
                dbcRaw[IDX_FLOW_SENSOR_DRIVE]   = (int)extractBitsLE(data, 16, 16);
                dbcRaw[IDX_FLOW_SENSOR_MOTOR]   = (int)extractBitsLE(data, 0, 16);
                break;

            // --- Trames a ID etendu (29 bits) du chargeur ---
            // Les ID du DBC (2566869222 etc.) portent un marqueur "etendu" sur
            // les bits hauts (convention Vector/CANdb++) ; une fois masques a
            // 29 bits (& 0x1FFFFFFF), ce sont les valeurs ci-dessous, qui
            // correspondent aux vrais ID lus sur le bus (format J1939).

            case 0x18FF50E6UL:   // charger_heartbeat (2566869222 masque)
                dbcRaw[IDX_CHARGER_HEARTBEAT] = (int)extractBitsLE(data, 0, 16);
                break;

            case 0x1806E5F4UL:   // charger_setting (2550588916 masque)
                dbcRaw[IDX_CHARGER_MAX_VOLTAGE_HIGH] = (int)extractBitsLE(data, 0, 8);
                dbcRaw[IDX_CHARGER_MAX_CURRENT_HIGH] = (int)extractBitsLE(data, 16, 8);
                dbcRaw[IDX_CHARGER_CONTROL]           = (int)extractBitsLE(data, 32, 1);
                dbcRaw[IDX_CHARGER_MAX_VOLTAGE_LOW]   = (int)extractBitsLE(data, 8, 8);
                dbcRaw[IDX_CHARGER_MAX_CURRENT_LOW]   = (int)extractBitsLE(data, 24, 8);
                break;

            case 0x18FF50E5UL:   // charger_info (2566869221 masque)
                dbcRaw[IDX_CHARGER_HARDWARE_FAILURE]      = (int)extractBitsLE(data, 32, 1);
                dbcRaw[IDX_CHARGER_OVER_TEMP]              = (int)extractBitsLE(data, 33, 1);
                dbcRaw[IDX_CHARGER_WRONG_INPUT_VOLTAGE]    = (int)extractBitsLE(data, 34, 1);
                dbcRaw[IDX_CHARGER_BATTERY_UNDETECTED]     = (int)extractBitsLE(data, 35, 1);
                dbcRaw[IDX_CHARGER_COMM_ERROR]              = (int)extractBitsLE(data, 36, 1);
                dbcRaw[IDX_CHARGER_CURRENT_OUT_HIGH]        = (int)extractBitsLE(data, 16, 8);
                dbcRaw[IDX_CHARGER_CURRENT_OUT_LOW]         = (int)extractBitsLE(data, 24, 8);
                dbcRaw[IDX_CHARGER_VOLTAGE_OUT_HIGH]        = (int)extractBitsLE(data, 0, 8);
                dbcRaw[IDX_CHARGER_VOLTAGE_OUT_LOW]         = (int)extractBitsLE(data, 8, 8);
                break;

            default:
                break;
        }
    }
}

// Copie les valeurs brutes decodees depuis le DBC dans sensorsOut[].
// N'ecrit PAS les indices legacy (0-9), voir commentaire dans capteurs.h.
void lireValeursDBC(int sensorsOut[NB_SENSOR]) {
    for (int i = IDX_BATTERY_MAX_TEMP; i < NB_SENSOR; i++) {
        sensorsOut[i] = dbcRaw[i];
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