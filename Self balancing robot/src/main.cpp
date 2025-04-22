/*
  TRAVELERS version DELTA 
  ROLIN@2024
  
  Basé sur le code de controle COSTE@2024 et réception I2C TOURON@2022

  Ce programme contrôle un microcontrôleur ESP32 qui interfère avec deux ESCs 
  (Electronic Speed Controllers) et des moteurs via la communication I2C.
*/
#include <Wire.h>
#include <MPU6050.h>
MPU6050 mpu;

// global variable 
float* RateRoll = new float(0);
float* RatePitch = new float(0);
float* RateYaw = new float(0);
float* AngleRoll= new float(0);
float* AnglePitch= new float(0);
float* AngleYaw= new float(0);
float* AccX= new float(0);
float* AccY= new float(0);
float* AccZ= new float(0);


bool checkMPU() {
    bool initialized = mpu.testConnection();
    if (initialized) {
        Serial.println("MPU6050 detected and initialized!");
        return true;
    } else {
        Serial.println("MPU6050 not detected! Check connections...");
        return false;
    }
}



void gyro_signal(){
  // Low pass filter 
  Wire.beginTransmission(0x68);
  Wire.write(0x1A);
  Wire.write(0x05);
  Wire.endTransmission();

  // configuration of the sensor
  Wire.beginTransmission(0x68);
  Wire.write(0x1C);
  Wire.write(0x10);
  Wire.endTransmission();
  Wire.beginTransmission(0x68);
  Wire.write(0x3B);
  Wire.endTransmission();

  // ask values 
  Wire.requestFrom(0x68,6);
  
  int16_t AXC = Wire.read() << 8 |Wire.read();
  int16_t AYC = Wire.read() << 8 |Wire.read();
  int16_t AZC = Wire.read() << 8 |Wire.read();


  // configuration of the gyroscope 
  Wire.beginTransmission(0x68);
  Wire.write(0x1B);
  Wire.write(0x8);
  Wire.endTransmission();
  Wire.beginTransmission(0x68);
  Wire.write(0x43);
  Wire.endTransmission();
  //ask values 
  Wire.requestFrom(0x68,6);
  
  int16_t GyroX = Wire.read() << 8 | Wire.read();
  int16_t GyroY = Wire.read() << 8 | Wire.read();
  int16_t GyroZ = Wire.read() << 8 | Wire.read();

  *RateRoll = (float)(GyroX/65.5);
  *RatePitch = (float)GyroX/65.5;
  *RateYaw = (float)GyroX/65.5;

  *AccX = (float)AXC/4096;
  *AccY = (float)AYC/4096;
  *AccZ = (float)AZC/4096;

  // global angle : 

  *AngleRoll = atan(*AccY/sqrt(*AccX * *AccX + *AccZ * *AccZ)) * 180.0/M_PI;
  *AnglePitch = atan(-1 * *AccX/sqrt(*AccY * *AccY + *AccZ * *AccZ)) * 180.0/M_PI;
}

void setup() {
    Serial.begin(115200);
    Wire.setClock(400000);
    Wire.begin();
    delay(250);
    Wire.beginTransmission(0x68);
    Wire.write(0x6B);
    Wire.write(0x00);
    Wire.endTransmission();
}

void loop() {
    gyro_signal();
    Serial.print(*AccX);
    Serial.print("  ");
    Serial.print(*AccY);
    Serial.print("  ");
    Serial.print(*AccZ);
    Serial.print("  ");

    Serial.print(*AngleRoll);
    Serial.print("   ");
    Serial.print(*AnglePitch);
    Serial.println("   ");

    delay(50); // Court délai pour la stabilité

}
