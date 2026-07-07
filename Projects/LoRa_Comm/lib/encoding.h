#ifndef ENCODING_H
#define ENCODING_H


#include <stdint.h>
#include <stdbool.h>
#include <RadioLib.h>


#define NB_SENSOR   10
#define SENSOR_NAME_MAX_LEN 16

#define CAN_SENSOR_ID_MIN 0x410
#define CAN_SENSOR_ID_MAX 0x45F



int data_formater_tab[NB_SENSOR] = {0};

/* à faire 
index | capteur
1 | x
2 | y
3 | z
etc...
*/

typedef struct
{
    char name[SENSOR_NAME_MAX_LEN];
    int threshold;
    uint8_t priority;
    int last_sent;
} SensorConfig;

class DeltaEncoder
{
public:
    DeltaEncoder();

    // Initialise tous les capteurs avec les valeurs actuelles.
    //param:
    //- sensors: tableau de valeurs des capteurs
    void init(const int sensors[]);

    // Encode les valeurs des capteurs selon les configurations.
    // Param:
    // - sensors: tableau de valeurs des capteurs
    // - out_index_capteur: tableau de sortie pour les index des capteurs modifiés
    // - out_value_capteur: tableau de sortie pour les valeurs des capteurs modifiés
    // - Retourne le nombre de capteurs modifiés
    uint16_t sorting(const int sensors[], uint8_t out_index_capteur[], int diff_capteur[]);
    int* data_formater(uint16_t nb_donnees, uint8_t output_index[], int output_diff[]);

    // sets and gets
    void setName(uint8_t sensorNumber, const char sensorName[]);
    void setThreshold(uint8_t sensorNumber, int threshold);
    void setPriority(uint8_t sensorNumber, int isPriority);
    SensorConfig getConfigByIndex(uint8_t index) const;

private:
    int previousValues[NB_SENSOR];
    SensorConfig configs[NB_SENSOR];
    bool isInitialized;
};

// Returns -1 if the CAN ID is outside [0x410, 0x45F].
int16_t sensorIndexFromCanId(uint16_t canId);

// Returns RadioLib status code (RADIOLIB_ERR_NONE on success).
int send_data(int data, SX1262& radio);


#endif