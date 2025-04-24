#include <Wire.h>
#include <MPU6050.h>

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

float* GyroRoll = new float(0);
float* GyroPitch = new float(0);


float* globalAngleRoll = new float(0);


unsigned long last_time =0;
unsigned long now = 0;


void gyro_signal(){
  // Low pass filter 
  Wire.beginTransmission(0x68);
  Wire.write(0x1A);
  Wire.write(0x05);
  Wire.endTransmission();

  // configuration of the accel
  Wire.beginTransmission(0x68);
  Wire.write(0x1C);
  Wire.write(0x10);
  Wire.endTransmission();
  Wire.beginTransmission(0x68);
  Wire.write(0x3B);
  Wire.endTransmission();
  Wire.requestFrom(0x68,6);

  //value of the accel
  int16_t AXC = Wire.read() << 8 |Wire.read();
  int16_t AYC = Wire.read() << 8 |Wire.read();
  int16_t AZC = Wire.read() << 8 |Wire.read();

  *AccX = (float)AXC/4096;
  *AccY = (float)AYC/4096;
  *AccZ = (float)AZC/4096;

  // global angle - accel angle
  *AngleRoll = atan(*AccY/sqrt(*AccX * *AccX + *AccZ * *AccZ))*180/PI;// atan2(*AccX,*AccZ)*180/PI;
  *AnglePitch = atan(*AccX/sqrt(*AccY * *AccY + *AccZ * *AccZ))*180/PI;//atan2(-*AccX,sqrt(*AccY * *AccY +*AccZ * *AccZ))*180 / PI;

  // Gyroscope
  
  // configuration of the gyroscope 
  Wire.beginTransmission(0x68);
  Wire.write(0x1B);
  Wire.write(0x8);
  Wire.endTransmission();
  Wire.beginTransmission(0x68);
  Wire.write(0x43);
  Wire.endTransmission();
  Wire.requestFrom(0x68,6);

  // value of gyro 
  int16_t GyroX = Wire.read() << 8 | Wire.read();
  int16_t GyroY = Wire.read() << 8 | Wire.read();
  int16_t GyroZ = Wire.read() << 8 | Wire.read();

    // global rat - gyro angle
  *RateRoll = (float)(GyroX/65.5);
  *RatePitch = (float)GyroX/65.5;
  *RateYaw = (float)GyroX/65.5;

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
    now = millis();
    float dt = (now -last_time)/1000;
    last_time = now;
    Serial.print(*AngleRoll);
    Serial.print("   ");
    Serial.print(*AnglePitch);
    Serial.print("   ");
    *GyroRoll +=*RateRoll*dt;
    *GyroPitch +=*RatePitch*dt;


    *globalAngleRoll = (0.02*(*AngleRoll) + 0.98*(*GyroRoll));
    Serial.print(*globalAngleRoll);
    Serial.println("   ");

    delay(50); // Court délai pour la stabilité

}
