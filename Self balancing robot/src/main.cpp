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
float acc_angle_y,acc_angle_x;
float total_angle_x, total_angle_y;
float gyroXerror = 0.07;
float gyroYerror = 0.03;
float gyroZerror = 0.01;


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
  //  Serial.printf("dt:%.f  now:%.lu  lastSignal:%.lu  gyroX:%.1f  \n" ,RAD_TO_DEG*(float)dt/1000, now, lastSignalTime, gyroX);

  // Remove threshold conditions for debugging
  float dt_s = (float)dt*0.001f;
  gyroX += gx* dt_s;
  gyroY += gy *dt_s;
  gyroZ += gz* dt_s;

  // get the accelerometer values 
  accX = accel.acceleration.x;
  accY = accel.acceleration.y;
  accZ = accel.acceleration.z;

  // Acceleration angles 
  acc_angle_x = (atan((accY)/sqrt(pow(accX,2) + pow(accY,2))));
  acc_angle_y = (atan(-1*(accX)/sqrt(pow(accY,2) + pow(accZ,2))));

  // Total angle and filter 
  total_angle_x = 0.98f * (total_angle_x + gx*dt_s) + 0.02f * acc_angle_x;
  total_angle_y = 0.98f * (total_angle_y + gy*dt_s) + 0.02f * acc_angle_y;

  lastSignalTime = now;
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
    localReadings["gyroX"] = String(gyroX);
    localReadings["gyroY"] = String(gyroY);
    localReadings["gyroZ"] = String(gyroZ);
    return JSON.stringify(localReadings);
}

String getAccReadings(){
    JSONVar localReadings;
    localReadings["accX"] = String(accX);
    localReadings["accY"] = String(accY);
    localReadings["accZ"] = String(accZ);
    return JSON.stringify(localReadings);
}



float PID(float current_angle){
  // Error - Proportionnal
  if(dt == 0){return 0;}
  float dt_s = (float)dt * 0.001f;
  float error = target_angle-current_angle;

  // Integral term
  integral += error*dt_s;
  integral = constrain(integral,-I_MAX, I_MAX);

  // Derivate turm 
  float derivate = (current_angle - oldvalue) / dt_s;
  oldvalue = current_angle;
  float result_PID = (error*Kp) + (integral*Ki) + (derivate*Kd);
  
  Serial.printf("P : %f , I : %f , D: %f\n",error,integral,derivate);
  return result_PID;
}

void setup() {
  // Initialisation of the modules
  Serial.begin(115200);
  initIMU();
  initWiFi();
  initLittleFS();

  pinMode(RIGHT_MOTOR_ENA_1,OUTPUT);
  pinMode(RIGHT_MOTOR_ENA_2,OUTPUT);
  pinMode(LEFT_MOTOR_ENA_1,OUTPUT);
  pinMode(LEFT_MOTOR_ENA_2,OUTPUT);
  pinMode(LEFT_MOTOR_PWM_1,OUTPUT);
  pinMode(RIGHT_MOTOR_PWM_1,OUTPUT);
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

void directionRight(bool direction){
  digitalWrite(RIGHT_MOTOR_ENA_1, (direction ? HIGH : LOW));
  digitalWrite(RIGHT_MOTOR_ENA_2, (direction ? LOW : HIGH));
}

void directionLeft(bool direction){
  digitalWrite(LEFT_MOTOR_ENA_1, (direction ? HIGH : LOW));
  digitalWrite(LEFT_MOTOR_ENA_2, (direction ? LOW : HIGH));
}


void loop() {
  if (!calibrated) {
      calibrateGyro();
  }

  // Acquisition of the angles
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

  // PID code

  float motorInput = PID(total_angle_x);
  if(abs(motorInput) < 5)
    motorInput == 0;
  Serial.printf("Total_angle_y : %f, PID response %f\n", total_angle_y,motorInput);
  directionLeft(motorInput > 0);
  directionRight(motorInput > 0 );
  analogWrite(RIGHT_MOTOR_PWM_1,constrain(abs(motorInput), 0, 255));
  analogWrite(LEFT_MOTOR_PWM_1,constrain(abs(motorInput), 0, 255));

}
