#include "../lib/encoding.h"

#include <string.h>
#include <stdlib.h>
#include <Arduino.h>
#include <RadioLib.h>


static const SensorConfig DefaultConfigs[NB_SENSOR] = {
    // name      | threshold | priority | last_sent
    {"sensor_0", 1, 0, 0},
    {"sensor_1", 1, 0, 0},
    {"sensor_2", 1, 0, 0},
    {"sensor_3", 1, 0, 0},
    {"sensor_4", 1, 0, 0},
    {"sensor_5", 1, 0, 0},
    {"sensor_6", 1, 0, 0},
    {"sensor_7", 1, 0, 0},
    {"sensor_8", 1, 0, 0},
    {"sensor_9", 1, 0, 0}
};

DeltaEncoder::DeltaEncoder() : isInitialized(false)
{
    memset(previousValues, 0, sizeof(previousValues));
    memcpy(configs, DefaultConfigs, sizeof(configs));
}

void DeltaEncoder::init(const int sensors[])
{
    for (uint8_t i = 0; i < NB_SENSOR; i++)
    {
        previousValues[i] = sensors[i];
    }

    isInitialized = true;
}

void DeltaEncoder::setName(uint8_t sensorNumber, const char sensorName[])
{
    strncpy(configs[sensorNumber].name, sensorName, SENSOR_NAME_MAX_LEN - 1);
    configs[sensorNumber].name[SENSOR_NAME_MAX_LEN - 1] = '\0';
}

void DeltaEncoder::setThreshold(uint8_t sensorNumber, int threshold)
{
    configs[sensorNumber].threshold = abs(threshold);
}

void DeltaEncoder::setPriority(uint8_t sensorNumber, int isPriorityValue)
{
    configs[sensorNumber].priority = (uint8_t)isPriorityValue & 0x03;
}

SensorConfig DeltaEncoder::getConfigByIndex(uint8_t index) const
{
    return configs[index];
}

uint16_t DeltaEncoder::sorting(const int sensors[], uint8_t out_index_capteur[], int diff_capteur[])
{
    uint16_t outIndex = 0;

    if (!isInitialized)
    {
        init(sensors);
    }
    
    for (uint8_t i = 0; i < NB_SENSOR; i++)
    {
        int old_value = previousValues[i];
        int now_value = sensors[i];

        int diff = now_value - old_value;

        if (abs(diff) >= configs[i].threshold)
        {
            configs[i].last_sent = now_value;
            //send_diff();
            out_index_capteur[outIndex] = i; // index capteur à i
            diff_capteur[outIndex] = diff; // valeur capteur à 

            previousValues[i] = now_value;
            outIndex++;
        }
    }
    return outIndex;
}

int* DeltaEncoder::data_formater(uint16_t nb_donnees, uint8_t output_index[], int output_diff[])
{
    int j = 0;
    for (int i = 0; i < nb_donnees; i++) 
    {
        data_formater_tab[j++] = output_index[i];
        data_formater_tab[j++] = output_diff[i] & 0xFF;
    }
    return data_formater_tab;
}




int send_data(int data, SX1262& radio)
{
    return radio.transmit((uint8_t*)&data, sizeof(data));
}



int16_t sensorIndexFromCanId(uint16_t canId)
{
/*
Tous les IDs entre 0x410 et 0x45f (inclusif)
ID 4 : ecu_sensors
ID 6 : ecu_battery
ID 9 : ecu_modes
ID 81 : drive_speed_temp
ID 82 : drive_electric_1
ID 84 : drive_electric_2
*/

    if (canId < CAN_SENSOR_ID_MIN || canId > CAN_SENSOR_ID_MAX)
    {
        return -1;
    }

    return (int16_t)(canId - CAN_SENSOR_ID_MIN);
}

// à faire avec une vraie trame can CAN_data[0] = ID est juste pour l'idée
void decode_CAN(int CAN_data[], int ID, int valeur, int sensors[])
{
    ID = CAN_data[0];
    int16_t sensorIndex = sensorIndexFromCanId(ID);
    
    valeur = CAN_data[1];
    sensors[sensorIndex] = valeur;
}