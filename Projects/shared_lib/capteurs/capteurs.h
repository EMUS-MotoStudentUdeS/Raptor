#pragma once
#include <stdint.h>

// ============================================================================
//  DÉFINITION DES INDEX ET DU NOMBRE TOTAL DE CAPTEURS
//  L'émetteur et le récepteur utiliseront cette liste pour se synchroniser.
// ============================================================================
enum SensorIndex {
    IDX_TIRE = 0,             // Index 0 : "tire"
    IDX_BAT,                  // Index 1 : "bat"
    IDX_FUEL,                 // Index 2 : "fuel"
    IDX_SPEED,                // Index 3 : "speed"
    IDX_TEMP,                 // Index 4 : "temp"

    // --- Capteurs de réserve pour monter à 10 ---
    IDX_RESERVE_5,            // Index 5
    IDX_RESERVE_6,            // Index 6
    IDX_RESERVE_7,            // Index 7
    IDX_RESERVE_8,            // Index 8
    IDX_RESERVE_9,            // Index 9

    // ========================================================================
    //  SIGNAUX ISSUS DU DBC (raptor.dbc) - un index par SG_, valeur BRUTE
    //  (avant application du scale/offset du DBC, qui reste a appliquer cote
    //  affichage/dashboard). Regroupes par BO_ (trame CAN).
    // ========================================================================

    // --- BO_ 6 ecu_battery ---
    IDX_BATTERY_MAX_TEMP,
    IDX_BATTERY_MAX_VOLTAGE,
    IDX_BATTERY_MIN_VOLTAGE,

    // --- BO_ 1538 komodo ---
    IDX_KOMODO_HEARTBEAT_8,
    IDX_KOMODO_B1,
    IDX_KOMODO_B2,
    IDX_KOMODO_B3,
    IDX_KOMODO_B4,
    IDX_KOMODO_B5,
    IDX_KOMODO_B6,

    // --- BO_ 84 drive_electric_2 ---
    IDX_THROTTLE_REQUEST,
    IDX_TORQUE,
    IDX_SYSTEM_KEY_ON_TIME,
    IDX_ODOMETER_LOW,
    IDX_ODOMETER_HIGH,

    // --- BO_ 86 drive_status ---
    IDX_FAULT_CODE,
    IDX_FAULT_LEVEL,

    // --- BO_ 82 drive_electric_1 ---
    IDX_IQ_REF,
    IDX_I_MOTOR,
    IDX_I_BAT,
    IDX_V_BAT,

    // --- BO_ 81 drive_speed_temp ---
    IDX_KEY_SWITCH_VOLT,
    IDX_MOTOR_TEMP,
    IDX_INVERTER_TEMP,
    IDX_SPEED_REF,
    IDX_SPEED_MEASURE,

    // --- BO_ 3 ecu_fault_list ---
    IDX_BENDER_EARTH_FAULT,
    IDX_BENDER_IMD_FAULT,
    IDX_BENDER_SPEED_START_BAD,
    IDX_BENDER_SPEED_START_GOOD,
    IDX_BENDER_UV_FAULT,
    IDX_BENDER_HAPPY,
    IDX_BENDER_SHORT_CIRCUIT_FAULT,
    IDX_CLUSTER_CHARGING,
    IDX_CLUSTER_NOT_SAFE,
    IDX_CLUSTER_ENGAGED,
    IDX_CLUSTER_SAFE,
    IDX_BATTERY_OVER_VOLTAGE_FAULT,
    IDX_BENDER_FAULT,
    IDX_BATTERY_DISCONNECTION_FAULT,
    IDX_BATTERY_UNDER_VOLTAGE_FAULT,
    IDX_BATTERY_OVER_TEMP_FAULT,
    IDX_E_STOP,

    // --- BO_ 9 ecu_modes ---
    IDX_B_BUTTON,
    IDX_FORWARD_SWITCH,
    IDX_MODE3,
    IDX_MODE2,
    IDX_MODE1,

    // --- BO_ 4 ecu_sensors ---
    IDX_CURRENT_SENSOR,
    IDX_FLOW_SENSOR_DRIVE,
    IDX_FLOW_SENSOR_MOTOR,

    // --- BO_ 2566869222 charger_heartbeat (ID etendu masque : 0x18FF50E6) ---
    IDX_CHARGER_HEARTBEAT,

    // --- BO_ 2550588916 charger_setting (ID etendu masque : 0x1806E5F4) ---
    IDX_CHARGER_MAX_VOLTAGE_HIGH,
    IDX_CHARGER_MAX_CURRENT_HIGH,
    IDX_CHARGER_CONTROL,
    IDX_CHARGER_MAX_VOLTAGE_LOW,
    IDX_CHARGER_MAX_CURRENT_LOW,

    // --- BO_ 2566869221 charger_info (ID etendu masque : 0x18FF50E5) ---
    IDX_CHARGER_HARDWARE_FAILURE,
    IDX_CHARGER_OVER_TEMP,
    IDX_CHARGER_WRONG_INPUT_VOLTAGE,
    IDX_CHARGER_BATTERY_UNDETECTED,
    IDX_CHARGER_COMM_ERROR,
    IDX_CHARGER_CURRENT_OUT_HIGH,
    IDX_CHARGER_CURRENT_OUT_LOW,
    IDX_CHARGER_VOLTAGE_OUT_HIGH,
    IDX_CHARGER_VOLTAGE_OUT_LOW,

    // ========================================================================
    //  SIGNAUX ISSUS DU VRAI DBC "BMS2025_1" (1er module BMS physique).
    //  Confirme le 2026-08-05 par 5 exports DBC coherents (modules 1 a 5,
    //  IDs 1040-1119) - pas de module "0". Voir decodeBmsModule() dans
    //  capteurs.cpp.
    // ========================================================================

    // --- ID 1044 (0x414) BMS2025_1_Tboard ---
    IDX_BMS_TBOARD,

    // --- ID 1043 (0x413) BMS2025_1_T04a05 ---
    IDX_BMS_T04,
    IDX_BMS_T05,

    // --- ID 1042 (0x412) BMS2025_1_T00a03 ---
    IDX_BMS_T00,
    IDX_BMS_T01,
    IDX_BMS_T02,
    IDX_BMS_T03,

    // --- ID 1041 (0x411) BMS2025_1_V04a05 ---
    IDX_BMS_V04,
    IDX_BMS_V05,

    // --- ID 1040 (0x410) BMS2025_1_V00a03 ---
    IDX_BMS_V00,
    IDX_BMS_V01,
    IDX_BMS_V02,
    IDX_BMS_V03,

    // --- ID 1055 (0x41F) BMS2025_1_Status : HB (16 bits, Intel) + CRC
    //     (4 bits) + BC1-6 (2 bits chacun), CRC/BC1-6 en Motorola/@0 - voir
    //     extractBitsBE() dans capteurs.cpp.
    IDX_BMS_HB,
    IDX_BMS_CRC,
    IDX_BMS_BC1,
    IDX_BMS_BC2,
    IDX_BMS_BC3,
    IDX_BMS_BC4,
    IDX_BMS_BC5,
    IDX_BMS_BC6,

    // ========================================================================
    //  SIGNAUX ISSUS DU VRAI DBC "BMS2025_2" (2e module BMS physique).
    // ========================================================================

    // --- ID 1060 (0x424) BMS2025_2_Tboard ---
    IDX_BMS2_TBOARD,

    // --- ID 1059 (0x423) BMS2025_2_T04a05 ---
    IDX_BMS2_T04,
    IDX_BMS2_T05,

    // --- ID 1058 (0x422) BMS2025_2_T00a03 ---
    IDX_BMS2_T00,
    IDX_BMS2_T01,
    IDX_BMS2_T02,
    IDX_BMS2_T03,

    // --- ID 1057 (0x421) BMS2025_2_V04a05 ---
    IDX_BMS2_V04,
    IDX_BMS2_V05,

    // --- ID 1056 (0x420) BMS2025_2_V00a03 ---
    IDX_BMS2_V00,
    IDX_BMS2_V01,
    IDX_BMS2_V02,
    IDX_BMS2_V03,

    // --- ID 1071 (0x42F) BMS2025_2_Status : HB + CRC/BC1-6 (voir bloc BMS2025_1) ---
    IDX_BMS2_HB,
    IDX_BMS2_CRC,
    IDX_BMS2_BC1,
    IDX_BMS2_BC2,
    IDX_BMS2_BC3,
    IDX_BMS2_BC4,
    IDX_BMS2_BC5,
    IDX_BMS2_BC6,

    // ========================================================================
    //  SIGNAUX ISSUS DU VRAI DBC "BMS2025_3" (3e module BMS physique).
    // ========================================================================

    // --- ID 1076 (0x434) BMS2025_3_Tboard ---
    IDX_BMS3_TBOARD,

    // --- ID 1075 (0x433) BMS2025_3_T04a05 ---
    IDX_BMS3_T04,
    IDX_BMS3_T05,

    // --- ID 1074 (0x432) BMS2025_3_T00a03 ---
    IDX_BMS3_T00,
    IDX_BMS3_T01,
    IDX_BMS3_T02,
    IDX_BMS3_T03,

    // --- ID 1073 (0x431) BMS2025_3_V04a05 ---
    IDX_BMS3_V04,
    IDX_BMS3_V05,

    // --- ID 1072 (0x430) BMS2025_3_V00a03 ---
    IDX_BMS3_V00,
    IDX_BMS3_V01,
    IDX_BMS3_V02,
    IDX_BMS3_V03,

    // --- ID 1087 (0x43F) BMS2025_3_Status : HB + CRC/BC1-6 (voir bloc BMS2025_1) ---
    IDX_BMS3_HB,
    IDX_BMS3_CRC,
    IDX_BMS3_BC1,
    IDX_BMS3_BC2,
    IDX_BMS3_BC3,
    IDX_BMS3_BC4,
    IDX_BMS3_BC5,
    IDX_BMS3_BC6,

    // ========================================================================
    //  SIGNAUX ISSUS DU VRAI DBC "BMS2025_4" (4e module BMS physique).
    // ========================================================================

    // --- ID 1092 (0x444) BMS2025_4_Tboard ---
    IDX_BMS4_TBOARD,

    // --- ID 1091 (0x443) BMS2025_4_T04a05 ---
    IDX_BMS4_T04,
    IDX_BMS4_T05,

    // --- ID 1090 (0x442) BMS2025_4_T00a03 ---
    IDX_BMS4_T00,
    IDX_BMS4_T01,
    IDX_BMS4_T02,
    IDX_BMS4_T03,

    // --- ID 1089 (0x441) BMS2025_4_V04a05 ---
    IDX_BMS4_V04,
    IDX_BMS4_V05,

    // --- ID 1088 (0x440) BMS2025_4_V00a03 ---
    IDX_BMS4_V00,
    IDX_BMS4_V01,
    IDX_BMS4_V02,
    IDX_BMS4_V03,

    // --- ID 1103 (0x44F) BMS2025_4_Status : HB + CRC/BC1-6 (voir bloc BMS2025_1) ---
    IDX_BMS4_HB,
    IDX_BMS4_CRC,
    IDX_BMS4_BC1,
    IDX_BMS4_BC2,
    IDX_BMS4_BC3,
    IDX_BMS4_BC4,
    IDX_BMS4_BC5,
    IDX_BMS4_BC6,

    // ========================================================================
    //  SIGNAUX ISSUS DU VRAI DBC "BMS2025_5" (5e module BMS physique).
    // ========================================================================

    // --- ID 1108 (0x454) BMS2025_5_Tboard ---
    IDX_BMS5_TBOARD,

    // --- ID 1107 (0x453) BMS2025_5_T04a05 ---
    IDX_BMS5_T04,
    IDX_BMS5_T05,

    // --- ID 1106 (0x452) BMS2025_5_T00a03 ---
    IDX_BMS5_T00,
    IDX_BMS5_T01,
    IDX_BMS5_T02,
    IDX_BMS5_T03,

    // --- ID 1105 (0x451) BMS2025_5_V04a05 ---
    IDX_BMS5_V04,
    IDX_BMS5_V05,

    // --- ID 1104 (0x450) BMS2025_5_V00a03 ---
    IDX_BMS5_V00,
    IDX_BMS5_V01,
    IDX_BMS5_V02,
    IDX_BMS5_V03,

    // --- ID 1119 (0x45F) BMS2025_5_Status : HB + CRC/BC1-6 (voir bloc BMS2025_1) ---
    IDX_BMS5_HB,
    IDX_BMS5_CRC,
    IDX_BMS5_BC1,
    IDX_BMS5_BC2,
    IDX_BMS5_BC3,
    IDX_BMS5_BC4,
    IDX_BMS5_BC5,
    IDX_BMS5_BC6,

    // ========================================================================
    //  SIGNAUX SORTIS PAR LE DASH AIM (RaceStudio3, onglet "CAN Output",
    //  CAN1 @ 250 kbit/s) - photo recapitulative recue le 2026-08-16, config
    //  deja transmise au dash cote AiM. Chaque champ Byte X-Y est un mot de
    //  16 bits ; format big-endian assume (convention standard de sortie CAN
    //  AiM) - A VALIDER via BUSMASTER/PEAK une fois le dash cable sur le bus,
    //  meme demarche que pour le reste du decodage (voir extractAimWord16BE
    //  dans capteurs.cpp).
    // ========================================================================

    // --- 0x500 [20Hz] ---
    IDX_AIM_COOL_TEMP_MOTOR_OUT,
    IDX_AIM_COOL_TEMP_MOTOR_IN,
    IDX_AIM_TEMP_RAD_AVANT,
    IDX_AIM_DRV_TRANS_TEMP,

    // --- 0x501 [20Hz] ---
    IDX_AIM_IMU_LON_ACC,
    IDX_AIM_IMU_LAT_ACC,
    IDX_AIM_IMU_VER_ACC,

    // --- 0x502 [20Hz] ---
    IDX_AIM_IMU_ROLL_RATE,
    IDX_AIM_IMU_PITCH_RATE,
    IDX_AIM_IMU_YAW_RATE,

    // --- 0x503 [20Hz] : Byte0-1 encore en "STATIC VALUE 0" cote RaceStudio -
    //     capteur reserve, garde ici pour ne pas avoir a retoucher l'enum/le
    //     switch le jour ou l'utilisateur y assigne un vrai canal (renommer
    //     IDX_AIM_0x503_RESERVED_B01 + la chaine correspondante suffira).
    IDX_AIM_0x503_RESERVED_B01,
    IDX_AIM_GPS_SPEED,

    // --- 0x504 [20Hz] : entierement en "STATIC VALUE 0" cote RaceStudio ---
    IDX_AIM_0x504_RESERVED_B01,
    IDX_AIM_0x504_RESERVED_B23,

    // --- 0x505 [20Hz] : entierement en "STATIC VALUE 0" cote RaceStudio ---
    IDX_AIM_0x505_RESERVED_B01,
    IDX_AIM_0x505_RESERVED_B23,

    NB_SENSOR                 // Nombre total de capteurs (calcule automatiquement)
};

// Structure actuelle de la moto (4 variables physiques)
struct DonneesMoto {
    float temp_pneu;
    float pression;
    float temp_huile;
    float vitesse;
};

// Fonctions de gestion du bus CAN
bool initCAN();
void majDonneesCAN();
DonneesMoto lireCapteurs();
bool lireTrameCAN(uint32_t* id, uint8_t* data, uint8_t* len);   // id sur 32 bits : supporte les trames standard (11 bits) ET etendues (29 bits)
void diagCAN();          // <- la ligne qui te manque

// Recupere les valeurs BRUTES decodees depuis le DBC (indices IDX_BATTERY_MAX_TEMP
// et suivants). Les indices legacy (IDX_TIRE..IDX_RESERVE_9, 0-9) ne sont PAS
// touches par cette fonction : ils restent geres via lireCapteurs()/DonneesMoto.
void lireValeursDBC(int sensorsOut[NB_SENSOR]);

// Diagnostic : debit reel de la trame CAN 81 (drive_speed_temp), sans log
// par trame. Voir le commentaire au point de definition dans capteurs.cpp.
uint32_t getCompteurTrame81();
void resetCompteurTrame81();
uint32_t getCompteurTrame82();
void resetCompteurTrame82();

// Noms lisibles de chaque capteur, dans le meme ordre que l'enum SensorIndex.
// Source UNIQUE des noms : utilisee par l'emetteur (DeltaEncoder::setName, via
// une boucle) ET par le recepteur (cles JSON), pour rester strictement
// synchronises sans dupliquer les chaines dans les deux main.cpp.
extern const char* const kSensorNames[NB_SENSOR];