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
#define REG_TEC      0x1C
#define REG_EFLG     0x2D
#define REG_RXB0CTRL 0x60
#define REG_RXB0SIDH 0x61
#define REG_RXB0SIDL 0x62
#define REG_RXB0EID8 0x63
#define REG_RXB0EID0 0x64
#define REG_RXB0DLC  0x65
#define REG_RXB0D0   0x66
#define REG_RXB1CTRL 0x70
#define REG_RXB1SIDH 0x71
#define REG_RXB1SIDL 0x72
#define REG_RXB1EID8 0x73
#define REG_RXB1EID0 0x74
#define REG_RXB1DLC  0x75
#define REG_RXB1D0   0x76

#define CANINTF_RX0IF 0x01
#define CANINTF_RX1IF 0x02

#define SIDL_IDE_BIT 0x08   // RXB0SIDL bit3 : 1 = trame etendue (29 bits), 0 = standard (11 bits)

SPIClass spiCAN(HSPI);

// Données courantes, mises à jour au fil des trames CAN reçues
static DonneesMoto donneesCourantes = {0.0f, 0.0f, 0.0f, 0.0f};

// Valeurs brutes decodees depuis le DBC (indices IDX_BATTERY_MAX_TEMP et
// suivants). Les indices legacy (0-9) ne sont jamais ecrits ici.
static int dbcRaw[NB_SENSOR] = {0};

// Diagnostic : nombre de trames CAN ID 81 (drive_speed_temp - contient
// speed_ref/speed_measure/motor_temp/inverter_temp) recues depuis le dernier
// resetCompteurTrame81(). Sert a mesurer le vrai debit source de cette trame
// precise sans reactiver le log verbeux par trame (qui ralentissait le cycle
// d'envoi, voir majDonneesCAN()).
static uint32_t compteurTrame81 = 0;

uint32_t getCompteurTrame81() {
    return compteurTrame81;
}

void resetCompteurTrame81() {
    compteurTrame81 = 0;
}

// Meme diagnostic que compteurTrame81, pour la trame 82 (drive_electric_1 -
// contient v_bat/i_bat/iq_ref/i_motor).
static uint32_t compteurTrame82 = 0;

uint32_t getCompteurTrame82() {
    return compteurTrame82;
}

void resetCompteurTrame82() {
    compteurTrame82 = 0;
}

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

    "bms_tboard",
    "bms_t04", "bms_t05",
    "bms_t00", "bms_t01", "bms_t02", "bms_t03",
    "bms_v04", "bms_v05",
    "bms_v00", "bms_v01", "bms_v02", "bms_v03",
    "bms_hb", "bms_crc", "bms_bc1", "bms_bc2", "bms_bc3", "bms_bc4", "bms_bc5", "bms_bc6",

    "bms2_tboard",
    "bms2_t04", "bms2_t05",
    "bms2_t00", "bms2_t01", "bms2_t02", "bms2_t03",
    "bms2_v04", "bms2_v05",
    "bms2_v00", "bms2_v01", "bms2_v02", "bms2_v03",
    "bms2_hb", "bms2_crc", "bms2_bc1", "bms2_bc2", "bms2_bc3", "bms2_bc4", "bms2_bc5", "bms2_bc6",

    "bms3_tboard",
    "bms3_t04", "bms3_t05",
    "bms3_t00", "bms3_t01", "bms3_t02", "bms3_t03",
    "bms3_v04", "bms3_v05",
    "bms3_v00", "bms3_v01", "bms3_v02", "bms3_v03",
    "bms3_hb", "bms3_crc", "bms3_bc1", "bms3_bc2", "bms3_bc3", "bms3_bc4", "bms3_bc5", "bms3_bc6",

    "bms4_tboard",
    "bms4_t04", "bms4_t05",
    "bms4_t00", "bms4_t01", "bms4_t02", "bms4_t03",
    "bms4_v04", "bms4_v05",
    "bms4_v00", "bms4_v01", "bms4_v02", "bms4_v03",
    "bms4_hb", "bms4_crc", "bms4_bc1", "bms4_bc2", "bms4_bc3", "bms4_bc4", "bms4_bc5", "bms4_bc6",

    "bms5_tboard",
    "bms5_t04", "bms5_t05",
    "bms5_t00", "bms5_t01", "bms5_t02", "bms5_t03",
    "bms5_v04", "bms5_v05",
    "bms5_v00", "bms5_v01", "bms5_v02", "bms5_v03",
    "bms5_hb", "bms5_crc", "bms5_bc1", "bms5_bc2", "bms5_bc3", "bms5_bc4", "bms5_bc5", "bms5_bc6",

    "aim_cool_tmpout", "aim_cool_tmpin", "aim_temp_radavt", "aim_drv_transtp",

    "aim_imu_lonacc", "aim_imu_latacc", "aim_imu_veracc",

    "aim_imu_rollrt", "aim_imu_pitchrt", "aim_imu_yawrt",

    "aim_fw_speed", "aim_gps_speed",

    "aim_tyretmp_f", "aim_tyretmp_r",

    "aim_linsusp_f", "aim_linsusp_r",

    "aim_brakepres_f", "aim_brakepres_r",
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

// Extraction Motorola/big-endian (@0), pour les signaux CRC/BC1-6 de la
// trame Status BMS - tout le reste de ce DBC est en Intel/@1 (extractBitsLE).
// Conversion bit motorola -> bit standard (LSB=0, comme extractBitsLE) :
// le bit ecrit dans le DBC designe le bit-en-octet compte MSB=0..LSB=7,
// contrairement a Intel ou le bit-en-octet est compte LSB=0..MSB=7. On
// convertit l'un vers l'autre (stdBit = octet*8 + (7 - bitEnOctetMotorola)),
// puis on extrait normalement. Verifie par recoupement interne sur les 7
// signaux de BMS2025_N_Status (CRC 19|4 + BC1..6 a 2 bits chacun) : cette
// formule est la seule des deux orientations testees qui les fait tenir
// exactement cote a cote sur 16 bits sans aucun chevauchement ni trou.
static uint64_t extractBitsBE(const uint8_t data[8], uint8_t startBitMotorola, uint8_t length) {
    uint8_t octet = startBitMotorola / 8;
    uint8_t bitEnOctet = startBitMotorola % 8;
    uint8_t stdBit = octet * 8 + (7 - bitEnOctet);
    return extractBitsLE(data, stdBit, length);
}

// ---------- Decodage des trames de sortie CAN du dash AiM (0x500-0x505) ----------
//
// Comme le reste des signaux DBC decodes ici, la sortie CAN custom du dash
// AiM (RaceStudio3 -> onglet "CAN Output") est en Intel/little-endian
// (octet de poids faible en premier) - confirme par l'utilisateur : meme
// convention que le reste du bus. Un mot de 16 bits ALIGNE sur les octets
// par canal. byteOffset designe le premier des 2 octets (ex: 0 pour
// Byte0-1, 2 pour Byte2-3...).
static int16_t extractAimWord16LE(const uint8_t data[8], uint8_t byteOffset) {
    return (int16_t)(((uint16_t)data[byteOffset + 1] << 8) | data[byteOffset]);
}

// ---------- Decodage generique des modules BMS2025_N ----------
//
// Chaque module BMS (BMS2025_1, _2, _3, _4, _5 - 5 modules physiques reels,
// confirmes le 2026-08-05 par 5 exports DBC coherents entre eux) emet les 6
// memes trames avec le meme layout de bits, seul l'ID CAN de base change :
// base = 1040 + 16*k (k = 0 pour le module 1, 1 pour le module 2, etc.). Au
// sein d'un module, les 6 trames sont a base+0 (V00a03), base+1 (V04a05),
// base+2 (T00a03), base+3 (T04a05), base+4 (Tboard) et base+15 (Status).
//
// kBmsModules associe explicitement chaque module a ses IDX_BMS*_* (definis
// dans capteurs.h) : pas de calcul d'offset sur l'enum, pour rester correct
// meme si l'ordre des IDX_BMS*_* venait a changer.
struct BmsModuleIndices {
    int tboard, t04, t05, t00, t01, t02, t03, v04, v05, v00, v01, v02, v03,
        hb, crc, bc1, bc2, bc3, bc4, bc5, bc6;
};

static const BmsModuleIndices kBmsModules[] = {
    { IDX_BMS_TBOARD, IDX_BMS_T04, IDX_BMS_T05, IDX_BMS_T00, IDX_BMS_T01, IDX_BMS_T02, IDX_BMS_T03,
      IDX_BMS_V04, IDX_BMS_V05, IDX_BMS_V00, IDX_BMS_V01, IDX_BMS_V02, IDX_BMS_V03,
      IDX_BMS_HB, IDX_BMS_CRC, IDX_BMS_BC1, IDX_BMS_BC2, IDX_BMS_BC3, IDX_BMS_BC4, IDX_BMS_BC5, IDX_BMS_BC6 },
    { IDX_BMS2_TBOARD, IDX_BMS2_T04, IDX_BMS2_T05, IDX_BMS2_T00, IDX_BMS2_T01, IDX_BMS2_T02, IDX_BMS2_T03,
      IDX_BMS2_V04, IDX_BMS2_V05, IDX_BMS2_V00, IDX_BMS2_V01, IDX_BMS2_V02, IDX_BMS2_V03,
      IDX_BMS2_HB, IDX_BMS2_CRC, IDX_BMS2_BC1, IDX_BMS2_BC2, IDX_BMS2_BC3, IDX_BMS2_BC4, IDX_BMS2_BC5, IDX_BMS2_BC6 },
    { IDX_BMS3_TBOARD, IDX_BMS3_T04, IDX_BMS3_T05, IDX_BMS3_T00, IDX_BMS3_T01, IDX_BMS3_T02, IDX_BMS3_T03,
      IDX_BMS3_V04, IDX_BMS3_V05, IDX_BMS3_V00, IDX_BMS3_V01, IDX_BMS3_V02, IDX_BMS3_V03,
      IDX_BMS3_HB, IDX_BMS3_CRC, IDX_BMS3_BC1, IDX_BMS3_BC2, IDX_BMS3_BC3, IDX_BMS3_BC4, IDX_BMS3_BC5, IDX_BMS3_BC6 },
    { IDX_BMS4_TBOARD, IDX_BMS4_T04, IDX_BMS4_T05, IDX_BMS4_T00, IDX_BMS4_T01, IDX_BMS4_T02, IDX_BMS4_T03,
      IDX_BMS4_V04, IDX_BMS4_V05, IDX_BMS4_V00, IDX_BMS4_V01, IDX_BMS4_V02, IDX_BMS4_V03,
      IDX_BMS4_HB, IDX_BMS4_CRC, IDX_BMS4_BC1, IDX_BMS4_BC2, IDX_BMS4_BC3, IDX_BMS4_BC4, IDX_BMS4_BC5, IDX_BMS4_BC6 },
    { IDX_BMS5_TBOARD, IDX_BMS5_T04, IDX_BMS5_T05, IDX_BMS5_T00, IDX_BMS5_T01, IDX_BMS5_T02, IDX_BMS5_T03,
      IDX_BMS5_V04, IDX_BMS5_V05, IDX_BMS5_V00, IDX_BMS5_V01, IDX_BMS5_V02, IDX_BMS5_V03,
      IDX_BMS5_HB, IDX_BMS5_CRC, IDX_BMS5_BC1, IDX_BMS5_BC2, IDX_BMS5_BC3, IDX_BMS5_BC4, IDX_BMS5_BC5, IDX_BMS5_BC6 },
};
static const uint8_t kNbBmsModules = sizeof(kBmsModules) / sizeof(kBmsModules[0]);

// Decode 'id' s'il appartient a un module BMS2025_N connu ; renvoie false
// sinon (id hors plage BMS, ou trame non geree du module -> laisse le switch
// principal de majDonneesCAN() traiter les autres ID).
static bool decodeBmsModule(uint32_t id, const uint8_t data[8]) {
    if (id < 1040) return false;
    uint32_t rel = id - 1040;
    uint32_t moduleIdx = rel / 16;
    uint32_t offset = rel % 16;
    if (moduleIdx >= kNbBmsModules) return false;

    const BmsModuleIndices& m = kBmsModules[moduleIdx];
    switch (offset) {
        case 0:   // V00a03
            dbcRaw[m.v00] = (int)extractBitsLE(data, 0, 16);
            dbcRaw[m.v01] = (int)extractBitsLE(data, 16, 16);
            dbcRaw[m.v02] = (int)extractBitsLE(data, 32, 16);
            dbcRaw[m.v03] = (int)extractBitsLE(data, 48, 16);
            return true;

        case 1:   // V04a05
            dbcRaw[m.v04] = (int)extractBitsLE(data, 0, 16);
            dbcRaw[m.v05] = (int)extractBitsLE(data, 16, 16);
            return true;

        case 2:   // T00a03
            dbcRaw[m.t00] = (int)extractBitsLE(data, 0, 16);
            dbcRaw[m.t01] = (int)extractBitsLE(data, 16, 16);
            dbcRaw[m.t02] = (int)extractBitsLE(data, 32, 16);
            dbcRaw[m.t03] = (int)extractBitsLE(data, 48, 16);
            return true;

        case 3:   // T04a05
            dbcRaw[m.t04] = (int)extractBitsLE(data, 0, 16);
            dbcRaw[m.t05] = (int)extractBitsLE(data, 16, 16);
            return true;

        case 4:   // Tboard
            dbcRaw[m.tboard] = (int)extractBitsLE(data, 0, 16);
            return true;

        case 15:   // Status - HB en Intel/@1 (0|16), CRC/BC1-6 en Motorola/@0
                   // (19|4 et 2 bits chacun) : voir extractBitsBE().
            dbcRaw[m.hb]  = (int)extractBitsLE(data, 0, 16);
            dbcRaw[m.crc] = (int)extractBitsBE(data, 19, 4);
            dbcRaw[m.bc1] = (int)extractBitsBE(data, 31, 2);
            dbcRaw[m.bc2] = (int)extractBitsBE(data, 29, 2);
            dbcRaw[m.bc3] = (int)extractBitsBE(data, 27, 2);
            dbcRaw[m.bc4] = (int)extractBitsBE(data, 25, 2);
            dbcRaw[m.bc5] = (int)extractBitsBE(data, 23, 2);
            dbcRaw[m.bc6] = (int)extractBitsBE(data, 21, 2);
            return true;

        default:
            return false;
    }
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

    // Bit timing 250 kbps @ crystal 8 MHz (BRP=1). Confirme le 2026-08-05 :
    // le bus BMS tourne bien a 250k aussi (comme le bus vehicule), pas 500k -
    // REC=128/EFLG=0xB a 500k, REC=0/EFLG=0x00 a 250k.
    spi_write(REG_CNF1, 0x01);
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
    // Le MCP2515 a DEUX buffers de reception (RXB0/RXB1), mais ce code n'a
    // longtemps lu que RXB0 - meme avec le rollover RXB0->RXB1 active
    // (RXB0CTRL), les trames tombees dans RXB1 n'etaient jamais relues, ce
    // qui finissait par declencher un vrai overflow (EFLG RX0OVR) pendant
    // les rafales (plusieurs modules BMS qui parlent presque en meme temps).
    // On verifie RXB0 d'abord, puis RXB1, un seul par appel (comme avant) -
    // mais les DEUX sont maintenant vraiment vides a chaque passage de
    // majDonneesCAN() (qui boucle sur lireTrameCAN() jusqu'a epuisement).
    uint8_t intf = spi_read(REG_CANINTF);

    uint8_t regSIDH, regSIDL, regEID8, regEID0, regDLC, regD0;
    uint8_t flagAEffacer;

    if (intf & CANINTF_RX0IF) {
        regSIDH = REG_RXB0SIDH; regSIDL = REG_RXB0SIDL;
        regEID8 = REG_RXB0EID8; regEID0 = REG_RXB0EID0;
        regDLC  = REG_RXB0DLC;  regD0   = REG_RXB0D0;
        flagAEffacer = CANINTF_RX0IF;
    } else if (intf & CANINTF_RX1IF) {
        regSIDH = REG_RXB1SIDH; regSIDL = REG_RXB1SIDL;
        regEID8 = REG_RXB1EID8; regEID0 = REG_RXB1EID0;
        regDLC  = REG_RXB1DLC;  regD0   = REG_RXB1D0;
        flagAEffacer = CANINTF_RX1IF;
    } else {
        return false;
    }

    uint8_t sidh = spi_read(regSIDH);
    uint8_t sidl = spi_read(regSIDL);
    uint32_t sid = ((uint32_t)sidh << 3) | (sidl >> 5);   // 11 bits (standard OU partie haute d'une trame etendue)

    if (sidl & SIDL_IDE_BIT) {
        // Trame etendue (29 bits) : SID (11 bits) + EID (18 bits, reparti sur
        // SIDL[1:0] + EID8 + EID0).
        uint8_t eid8 = spi_read(regEID8);
        uint8_t eid0 = spi_read(regEID0);
        uint32_t eid = ((uint32_t)(sidl & 0x03) << 16) | ((uint32_t)eid8 << 8) | eid0;
        *id = (sid << 18) | eid;
    } else {
        // Trame standard (11 bits)
        *id = sid;
    }

    *len = spi_read(regDLC) & 0x0F;
    if (*len > 8) *len = 8;
    for (uint8_t i = 0; i < *len; i++) data[i] = spi_read(regD0 + i);

    // On ne libere QUE le buffer qu'on vient de lire (pas l'autre flag) :
    // l'effacer aveuglement comme avant (0x00) aurait pu perdre la
    // notification d'une trame deja arrivee dans l'autre buffer.
    spi_write(REG_CANINTF, intf & ~flagAEffacer);
    return true;
}

// Vide toutes les trames en attente et met à jour donneesCourantes.
// À appeler souvent dans loop().
void majDonneesCAN() {
    uint32_t id;
    uint8_t data[8] = {0};   // zero-init : les signaux DBC supposent une trame de 8 octets complete
    uint8_t len;

    while (lireTrameCAN(&id, data, &len)) {
        // Log par trame retire : avec le trafic CAN reel de la moto (5 modules
        // BMS + drivetrain), le Serial.print() par trame ralentissait le cycle
        // d'envoi de ~200ms a ~370ms (mesure : 2.7 paquets/s au lieu de 5/s
        // vises). Voir diagCAN() pour un diagnostic global (REC/TEC/EFLG) qui
        // ne coute pas ce prix.

        // Modules BMS2025_N (N=1..5 actuellement) : layout de bits identique
        // pour tous, seul l'ID de base change -> decodage generique plutot
        // qu'un bloc switch/case duplique par module (voir decodeBmsModule).
        if (decodeBmsModule(id, data)) {
            continue;
        }

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
                compteurTrame82++;
                // Diagnostic ponctuel : horodatage exact de chaque arrivee,
                // pour voir si le pattern est vraiment en rafale ou juste un
                // artefact du comptage par seconde. Peu couteux car cette
                // trame precise n'arrive que ~1x/s (contrairement au log par
                // trame qu'on a enleve de majDonneesCAN()).
                Serial.print("CAN82 t=");
                Serial.println(millis());
                break;

            case 81:   // drive_speed_temp
                dbcRaw[IDX_KEY_SWITCH_VOLT] = signExtend(extractBitsLE(data, 48, 16), 16);
                dbcRaw[IDX_MOTOR_TEMP]      = (int)extractBitsLE(data, 0, 8);
                dbcRaw[IDX_INVERTER_TEMP]   = (int)extractBitsLE(data, 8, 8);
                dbcRaw[IDX_SPEED_REF]       = signExtend(extractBitsLE(data, 16, 16), 16);
                dbcRaw[IDX_SPEED_MEASURE]   = signExtend(extractBitsLE(data, 32, 16), 16);
                compteurTrame81++;
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

            // --- Sortie CAN du dash AiM (RaceStudio3, "Custom Protocol"
            // EMUS_ScreenOutput, CAN1 @ 250 kbit/s). Mots 16 bits alignes
            // sur les octets, Intel/little-endian (voir extractAimWord16LE),
            // layout confirme bit-exact par la vue detaillee RaceStudio3.
            case 0x500:
                dbcRaw[IDX_AIM_COOL_TEMP_MOTOR_OUT] = extractAimWord16LE(data, 0);
                dbcRaw[IDX_AIM_COOL_TEMP_MOTOR_IN]  = extractAimWord16LE(data, 2);
                dbcRaw[IDX_AIM_TEMP_RAD_AVANT]      = extractAimWord16LE(data, 4);
                dbcRaw[IDX_AIM_DRV_TRANS_TEMP]      = extractAimWord16LE(data, 6);
                break;

            case 0x501:
                dbcRaw[IDX_AIM_IMU_LON_ACC] = extractAimWord16LE(data, 0);
                dbcRaw[IDX_AIM_IMU_LAT_ACC] = extractAimWord16LE(data, 2);
                dbcRaw[IDX_AIM_IMU_VER_ACC] = extractAimWord16LE(data, 4);
                break;

            case 0x502:
                dbcRaw[IDX_AIM_IMU_ROLL_RATE]  = extractAimWord16LE(data, 0);
                dbcRaw[IDX_AIM_IMU_PITCH_RATE] = extractAimWord16LE(data, 2);
                dbcRaw[IDX_AIM_IMU_YAW_RATE]   = extractAimWord16LE(data, 4);
                break;

            case 0x503:
                dbcRaw[IDX_AIM_FRONT_WHEEL_SPEED] = extractAimWord16LE(data, 0);
                dbcRaw[IDX_AIM_GPS_SPEED]         = extractAimWord16LE(data, 2);
                break;

            case 0x504:
                dbcRaw[IDX_AIM_TYRE_TEMP_FRONT] = extractAimWord16LE(data, 0);
                dbcRaw[IDX_AIM_TYRE_TEMP_REAR]  = extractAimWord16LE(data, 2);
                break;

            case 0x505:
                dbcRaw[IDX_AIM_LIN_SUSP_FRONT] = extractAimWord16LE(data, 0);
                dbcRaw[IDX_AIM_LIN_SUSP_REAR]  = extractAimWord16LE(data, 2);
                break;

            case 0x506:
                dbcRaw[IDX_AIM_BRAKE_PRES_FRONT] = extractAimWord16LE(data, 0);
                dbcRaw[IDX_AIM_BRAKE_PRES_REAR]  = extractAimWord16LE(data, 2);
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

// Affiche les compteurs d'erreurs de réception/émission + EFLG.
// REC/TEC=0 et EFLG=0 en continu, sans aucune trame reçue -> bus vraiment
// silencieux (rien a lire : alimentation/câblage/nœuds inactifs, pas un
// probleme de bitrate).
// REC/TEC qui grimpent ou EFLG != 0 (bits d'erreur : bit-stuff, CRC, form
// error...) -> le bus parle mais le bitrate ne correspond pas (ou bruit).
void diagCAN() {
    Serial.print("[diag] REC=");
    Serial.print(spi_read(REG_REC));
    Serial.print(" TEC=");
    Serial.print(spi_read(REG_TEC));
    Serial.print(" EFLG=0x");
    Serial.println(spi_read(REG_EFLG), HEX);
}