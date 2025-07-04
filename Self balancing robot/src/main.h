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

// Constant MPU
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

float target_angle = 0;
float integral = 0;
float oldvalue = 0;

float Kp = 70.0f;
float Ki = 0.0f;
float Kd = 30.0f;
float motorInput =0;

// Definition of the Motor PIN : 
#define RIGHT_MOTOR_ENA_1 33
#define RIGHT_MOTOR_ENA_2 32
#define RIGHT_MOTOR_PWM_1 27
//#define RIGTH_MOTOR_PWM_2 18

#define LEFT_MOTOR_ENA_1 25
#define LEFT_MOTOR_ENA_2 26
#define LEFT_MOTOR_PWM_1 14
//#define LEFT_MOTOR_PWM_2 

#define MAX 255

const float INTEGRAL_MAX = 100.0f;

#endif
