#ifndef MAIN_H
#define MAIN_H

#include <Arduino.h>
#include <WiFi.h>
#include <Wire.h>
#include <AsyncTCP.h>
#include <Adafruit_MPU6050.h>
#include <ESPAsyncWebServer.h>
#include <Arduino_JSON.h>
#include "LittleFS.h"
// Constantes MPU
#define MPU_ADDR 0x68
#define ESP_ADDRESS 0x75
const char* ssid = "TelRobin";
const char* password = "robinestbeau";

// Timer variables
unsigned long lastTime = 0;  
unsigned long lastTimeTemperature = 0;
unsigned long lastTimeAcc = 0;
unsigned long gyroDelay = 50;
unsigned long accelerometerDelay = 200;

#endif
