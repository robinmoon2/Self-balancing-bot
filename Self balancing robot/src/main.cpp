#include "main.h"

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


// Create AsyncWebServer object on port 80
AsyncWebServer server(80);

// Create an Event Source on /events
AsyncEventSource events("/events");

// Json Variable to Hold Sensor Readings
JSONVar readings;


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

void initWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  Serial.println("");
  Serial.print("Connecting to WiFi...");
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(1000);
  }
  Serial.println("");
  Serial.println(WiFi.localIP());
}

String getGyroReadings(){
    // Utilise les variables globales RateRoll, RatePitch et RateYaw et ne relit pas les valeurs du gyroscope
    readings["gyroX"] = String(*RateRoll);
    readings["gyroY"] = String(*RatePitch);
    readings["gyroZ"] = String(*RateYaw);

    String jsonString = JSON.stringify(readings);
    return jsonString;
}

String getAccReadings(){
    // Utilise les variables globales AccX, AccY et AccZ et ne relit pas les valeurs de l'accéléromètre
    readings["accX"] = String(*AccX);
    readings["accY"] = String(*AccY);
    readings["accZ"] = String(*AccZ);

    String jsonString = JSON.stringify(readings);
    return jsonString;
}

String getTemperature(){
    // Retourne toujours la valeur "5" pour la température
    readings["temperature"] = "5";
    String jsonString = JSON.stringify(readings);
    return jsonString;
}

void setup() {
  // Handle Web Server
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(LittleFS, "/index.html", "text/html");
  });

  server.serveStatic("/", LittleFS, "/");

  server.on("/reset", HTTP_GET, [](AsyncWebServerRequest *request){
    RateRoll=0;
    RatePitch=0;
    RateYaw=0;
    request->send(200, "text/plain", "OK");
  });


  server.on("/resetX", HTTP_GET, [](AsyncWebServerRequest *request){
    RateRoll=0;
    request->send(200, "text/plain", "OK");
  });

  server.on("/resetY", HTTP_GET, [](AsyncWebServerRequest *request){
    RatePitch=0;
    request->send(200, "text/plain", "OK");
  });

  server.on("/resetZ", HTTP_GET, [](AsyncWebServerRequest *request){
    RateYaw=0;
    request->send(200, "text/plain", "OK");
  });

  // Handle Web Server Events
  events.onConnect([](AsyncEventSourceClient *client){
    if(client->lastId()){
      Serial.printf("Client reconnected! Last message ID that it got is: %u\n", client->lastId());
    }
    // send event with message "hello!", id current millis
    // and set reconnect delay to 1 second
    client->send("hello!", NULL, millis(), 10000);
  });
  server.addHandler(&events);

  server.begin();
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(400000);
  

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
  if ((millis() - lastTimeTemperature) > temperatureDelay) {
    // Send Events to the Web Server with the Sensor Readings
    events.send(getTemperature().c_str(),"temperature_reading",millis());
    lastTimeTemperature = millis();
  }

}
