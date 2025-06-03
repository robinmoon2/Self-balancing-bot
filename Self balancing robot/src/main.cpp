#include "main.h"

// Global variables: replaced pointer declarations with simple floats.
float RateRoll = 0;
float RatePitch = 0;
float GyroRoll = 0;
float GyroPitch = 0;
float globalAngleRoll = 0;
float gyroBiasRoll = 0, gyroBiasPitch = 0, gyroBiasYaw = 0;
bool calibrated = false;

// Global variables: variable for the gyroscope and accelerometer
float gyroX,gyroY,gyroZ;
float accX, accY, accZ;
float gyroXerror = 0.07;
float gyroYerror = 0.03;
float gyroZerror = 0.01;
// Global variables: variable for the global acceleration and angle
float AngleRoll,AnglePitch,AngleYaw;

String gyroReading;
String accReading;


Adafruit_MPU6050 mpu;

// Global variables: variable for time differencies
unsigned long last_time =0;
unsigned long now = 0;
unsigned long dt =0;
unsigned long lastSignalTime = 0;
// Create AsyncWebServer object on port 80
AsyncWebServer server(80);

// Create an Event Source on /events
AsyncEventSource events("/events");

// Json Variable to Hold Sensor Readings
JSONVar readings;

/// ----------------------------------------------------------


// return the rad / s 
void gyro_signal(){
  sensors_event_t accel, gyro, temp;
  mpu.getEvent(&accel, &gyro, &temp);

  now = millis(); // get the time 
  dt = (now -lastSignalTime); // time differencies for the angle
  float gx = gyro.gyro.x - gyroBiasRoll;
  float gy = gyro.gyro.y - gyroBiasPitch;
  float gz = gyro.gyro.z - gyroBiasYaw;
  
  // Debug: print raw values
  Serial.printf("dt:%.f  now:%.lu  lastSignal:%.lu  gyroX:%.1f  \n" ,RAD_TO_DEG*(float)dt/1000, now, lastSignalTime, gyroX);

  // Remove threshold conditions for debugging
  gyroX += gx* ((float)dt/1000 * RAD_TO_DEG);
  gyroY += gy * ((float)dt/1000 * RAD_TO_DEG);
  gyroZ += gz* ((float)dt/1000 * RAD_TO_DEG);
  // get the accelerometer values 
  accX = accel.acceleration.x;
  accY = accel.acceleration.y;
  accZ = accel.acceleration.z;
  lastSignalTime = now;

  /*

  unsigned long currentTime = millis();
  float dtSignal = (currentTime - lastSignalTime) / 1000.0f; // s
  lastSignalTime = currentTime;
  
  gyroX += gyro.gyro.x/50;
  gyroY += gyro.gyro.y/70;
  gyroZ += gyro.gyro.z/90;

  accX = accel.acceleration.x;
  accY = accel.acceleration.y;
  accZ = accel.acceleration.z;

  float r = gyro.gyro.x * RAD_TO_DEG;
  float p = gyro.gyro.y * RAD_TO_DEG;
  float y = gyro.gyro.z * RAD_TO_DEG;

  if (calibrated) {
      float correctedRateRoll = (r - gyroBiasRoll);
      float correctedRatePitch = (p - gyroBiasPitch);
      float correctedRateYaw = (y - gyroBiasYaw);

      float accAngleRoll = atan2f(accel.acceleration.y, accel.acceleration.z) * RAD_TO_DEG;
      float accAnglePitch = atan2f(-accel.acceleration.x, sqrtf(accel.acceleration.y*accel.acceleration.y +
                                                                  accel.acceleration.z*accel.acceleration.z)) * RAD_TO_DEG;
      float alpha = 0.98;
      AngleRoll   = alpha * (AngleRoll   + correctedRateRoll * dtSignal) + (1 - alpha) * accAngleRoll;
      AnglePitch  = alpha * (AnglePitch  + correctedRatePitch * dtSignal) + (1 - alpha) * accAnglePitch;
      AngleYaw   += correctedRateYaw * dtSignal;
  } else {
      AngleRoll  += r * dtSignal;
      AnglePitch += p * dtSignal;
      AngleYaw   += y * dtSignal;
  }*/
}

void calibrateGyro() {
    const int sampleCount = 1000;
    float sumRoll = 0, sumPitch = 0, sumYaw = 0;
    sensors_event_t accel, gyro, temp;
    for (int i = 0; i < sampleCount; i++) {
        mpu.getEvent(&accel, &gyro, &temp);
        sumRoll  += gyro.gyro.x;
        sumPitch += gyro.gyro.y;
        sumYaw   += gyro.gyro.z;
        delay(10);
    }
    gyroBiasRoll = sumRoll / sampleCount;
    gyroBiasPitch = sumPitch / sampleCount;
    gyroBiasYaw = sumYaw / sampleCount;
    calibrated = true;
    Serial.println("Gyro calibrated.");
}


/// initialisation of the IMU
void initIMU()
{
  if (!mpu.begin()) {
    Serial.println(F("MPU-6050 absent !"));
  }
  Serial.println(F("MPU-6050 prêt"));
}

void initWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi ..");
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print('.');
    delay(1000);
  }
  Serial.println(WiFi.localIP());
}

void initLittleFS() {
  if (!LittleFS.begin()) {
    Serial.println("An error has occurred while mounting LittleFS");
  }
  Serial.println("LittleFSmounted successfully");
}



String getGyroReadings(){
    JSONVar localReadings;
    localReadings["gyroX"] = String(gyroX*DEG_TO_RAD);
    localReadings["gyroY"] = String(gyroY*DEG_TO_RAD);
    localReadings["gyroZ"] = String(gyroZ*DEG_TO_RAD);
    return JSON.stringify(localReadings);
}

String getAccReadings(){
    JSONVar localReadings;
    localReadings["accX"] = String(accX);
    localReadings["accY"] = String(accY);
    localReadings["accZ"] = String(accZ);
    return JSON.stringify(localReadings);
}


void setup() {
  // Initialisation of the modules
  Serial.begin(115200);
  initIMU();
  initWiFi();
  initLittleFS();

  // Handle Web Server
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(LittleFS, "/index.html", "text/html");
  });

  server.serveStatic("/", LittleFS, "/");

  server.on("/reset", HTTP_GET, [](AsyncWebServerRequest *request){
    gyroX = 0;
    gyroY = 0;
    gyroZ = 0;
    request->send(200, "text/plain", "OK");
  });


  // server action for sending data 
  server.on("/resetX", HTTP_GET, [](AsyncWebServerRequest *request){
    gyroX = 0;
    request->send(200, "text/plain", "OK");
  });

  server.on("/resetY", HTTP_GET, [](AsyncWebServerRequest *request){
    gyroY = 0;
    request->send(200, "text/plain", "OK");
  });

  server.on("/resetZ", HTTP_GET, [](AsyncWebServerRequest *request){
    gyroZ = 0;
    request->send(200, "text/plain", "OK");
  });



  // Handle Web Server Events
  events.onConnect([](AsyncEventSourceClient *client){
    if(client->lastId()){
      Serial.printf("Client reconnected! Last message ID that it got is: %u\n", client->lastId());
    }
    client->send("hello!", NULL, millis(), 10000);
  });
  server.addHandler(&events);
  server.begin();
  delay(250);
}


void loop() {
  if (!calibrated) {
      calibrateGyro();
  }

  gyro_signal();
  // send the data to the server
  if ((millis() - lastTime) > gyroDelay) {
    // Send Events to the Web Server with the Sensor Readings
    events.send(getGyroReadings().c_str(),"gyro_readings",millis());
    lastTime = millis();
  }
  if ((millis() - lastTimeAcc) > accelerometerDelay) {
    // Send Events to the Web Server with the Sensor Readings
    events.send(getAccReadings().c_str(),"accelerometer_readings",millis());
    lastTimeAcc = millis();
  }  
}
