#ifndef MAIN_H
#define MAIN_H

#include <Arduino.h>

// Constantes MPU
#define MPU_ADDR 0x68
#define ESP_ADDRESS 0x75
// Déclarations des fonctions
uint8_t readMPURegister(uint8_t reg);
int myFunction(int x, int y);



#endif
