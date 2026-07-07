#pragma once

// SPI & LoRa (SX1262)
#define SS    8
#define MOSI  10
#define MISO  11
#define SCK   9

// LoRa control pins
#define RST_LoRa   12
#define BUSY_LoRa  13
#define DIO1       14

// OLED
#define SDA_OLED   17
#define SCL_OLED   18
#define RST_OLED   21

// Vext (alimente l'OLED et d'autres périphériques)
#define Vext       36

// LED
#define LED        35

// Bouton BOOT
#define PRG_BTN    0