#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>

// Define motor pins
const int motor1Pin = 3;  
const int motor2Pin = 6; 

// Define PID constants
const float Kp = 1.2;
const float Ki = 0.40;
const float Kd = 0.002;

// Define Kalman filter variables
float angle; 
float bias = 0.0; 
float P[2][2] = {{1, 0}, {0, 1}};
const float Q_angle = 0.001; 
const float R_measure = 0.03; 
// define other variables
unsigned long prevTime = 0;
float dt = 0.02; 
float integral = 0.0;
float prev_error = 0.0;
float target_angle = 0.0;

Adafruit_BNO055 imu = Adafruit_BNO055();

void setup() {
  Serial.begin(115200);
  
  // Initialize IMU
  if(!imu.begin())
  {
    Serial.println("IMU not initialized");
    while(1);
  }
  pinMode(motor1Pin, OUTPUT);
  pinMode(motor2Pin, OUTPUT);
}

void loop() {
  // time since last loop 
  unsigned long currentTime = millis();
  float elapsedTime = (currentTime - prevTime) / 1000.0; 
  prevTime = currentTime;

  // IMU data
  sensors_event_t event;
  imu.getEvent(&event);
  float accel_angle = atan2(event.acceleration.y, event.acceleration.z) * RAD_TO_DEG;

  // Kalman filter
  float Pdot[4] = {0, 0, 0, 0};
  float x[2] = {0, 0}; 

  // Predict
  x[0] += dt * (event.gyro.x - bias);
  x[1] += dt * Q_angle;
  P[0][0] += dt * (dt * P[1][1] - P[0][1] - P[1][0] + Q_angle);
  P[0][1] -= dt * P[1][1];
  P[1][0] -= dt * P[1][1];
  P[1][1] += dt * R_measure;

  // Update
  float y = accel_angle - x[0];
  float S = P[0][0] + R_measure;
  float K[2] = {P[0][0] / S, P[1][0] / S};

  x[0] += K[0] * y;
  x[1] += K[1] * y;
  P[0][0] -= K[0] * P[0][0];
  P[0][1] -= K[0] * P[0][1];
  P[1][0] -= K[1] * P[0][0];
  P[1][1] -= K[1] * P[0][1];

  // PID control
  float error = x[0] - target_angle;
  integral += error * elapsedTime;
  float derivative = (error - prev_error) / elapsedTime;
  float pid_output = Kp * error + Ki * integral + Kd * derivative;

  
  int motorSpeed1 = 127 + pid_output;
  int motorSpeed2 = 127 - pid_output;


  analogWrite(motor1Pin, motorSpeed1);
  analogWrite(motor2Pin, motorSpeed2);

  
  prev_error = error;

  delay(20); 
}

