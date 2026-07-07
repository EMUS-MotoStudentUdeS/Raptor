#include "../lib/encoding.h"

#include <string.h>
#include <stdlib.h>
#include <Arduino.h>

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

void DeltaEncoder::init(const uint16_t sensors[])
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

void DeltaEncoder::setThreshold(uint8_t sensorNumber, uint16_t threshold)
{
    configs[sensorNumber].threshold = threshold;
}

void DeltaEncoder::setPriority(uint8_t sensorNumber, int isPriorityValue)
{
    configs[sensorNumber].priority = (uint8_t)isPriorityValue & 0x03;
}

SensorConfig DeltaEncoder::getConfigByIndex(uint8_t index) const
{
    return configs[index];
}

uint16_t DeltaEncoder::sorting(const uint16_t sensors[], uint8_t out_index_capteur[], uint8_t diff_capteur[])
{
    uint16_t outIndex = 0;

    if (!isInitialized)
    {
        init(sensors);
    }
    
    for (uint8_t i = 0; i < NB_SENSOR; i++)
    {
        uint16_t old_value = previousValues[i];
        uint16_t now_value = sensors[i];

        uint16_t diff = abs(old_value - now_value);

        if (diff >= configs[i].threshold)
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
 

int16_t sensorIndexFromCanId(uint16_t canId)
{
    if (canId < CAN_SENSOR_ID_MIN || canId > CAN_SENSOR_ID_MAX)
    {
        return -1;
    }

    return (int16_t)(canId - CAN_SENSOR_ID_MIN);
}


void decode_CAN(int CAN_data, int ID, int valeur)
{
    
    
}

