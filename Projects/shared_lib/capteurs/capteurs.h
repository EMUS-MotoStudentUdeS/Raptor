#pragma once
#include <stdint.h>

struct DonneesMoto {
    float temp_pneu;
    float pression;
    float temp_huile;
    float vitesse;
};

bool initCAN();
void majDonneesCAN();
DonneesMoto lireCapteurs();
bool lireTrameCAN(uint16_t* id, uint8_t* data, uint8_t* len);
void diagCAN();          // <- la ligne qui te manque