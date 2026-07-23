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

// Noms lisibles de chaque capteur, dans le meme ordre que l'enum SensorIndex.
// Source UNIQUE des noms : utilisee par l'emetteur (DeltaEncoder::setName, via
// une boucle) ET par le recepteur (cles JSON), pour rester strictement
// synchronises sans dupliquer les chaines dans les deux main.cpp.
extern const char* const kSensorNames[NB_SENSOR];