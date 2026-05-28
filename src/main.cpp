#include <Arduino.h>
#include <Wire.h>
#include <Servo.h>
#include "MPU6050.h"
#include "esp32-hal-ledc.h"
#include "esp32-hal-gpio.h"
#include "esp32-hal-i2c.h"
#include <Adafruit_MMA8451.h>
#include <Adafruit_Sensor.h>


// Hardware Pin Definitions (Adjust ESP32 pins as needed)
//Note: GPIO 6-11 are reserved for flash memory on most ESP32 boards
//(So dont use those)

//Pin definitions
const int trigPin = 4;
const int echoPin = 15;

// TB6612 Motor Driver Pins
#define PWMA 18
#define AIN1 23
#define AIN2 22
#define PWMB 19
#define BIN1 21
#define BIN2 5
#define STBY 2

// MMA8451 Accelerometer Pins (I2C)
#define SDA_PIN 33
#define SCL_PIN 32

//Global objecrs
MPU6050 imu;
Adafruit_MMA8451 mma = Adafruit_MMA8451();

float yaw = 0;
unsigned long lastTime = 0;

void setup() {
    Serial.begin(115200); //Used to communicate w/ ESP32 (Sensor data output)
    Serial2.begin(115200, SERIAL_8N1, 16, 17); //Used to communicate w/ Jetson (Command input)

    // Ultrasonic
    pinMode(trigPin, OUTPUT);
    pinMode(echoPin, INPUT);

    // Motor pins
    pinMode(AIN1, OUTPUT);
    pinMode(AIN2, OUTPUT);
    pinMode(BIN1, OUTPUT);
    pinMode(BIN2, OUTPUT);
    pinMode(STBY, OUTPUT);
    digitalWrite(STBY, HIGH);

    // PWM channels
    ledcSetup(0, 1000, 8);
    ledcAttachPin(PWMA, 0);
    ledcSetup(1, 1000, 8);
    ledcAttachPin(PWMB, 1);

    // IMU
    Wire.begin(SDA_PIN, SCL_PIN);
    imu.initialize();

    lastTime = micros();

    // MMA8451
    if (! mma.begin()) {
    Serial.println("Couldnt start");
    while (1);
    }
     Serial.println("MMA8451 found!");
  
    mma.setRange(MMA8451_RANGE_2_G);
  
    Serial.print("Range = "); Serial.print(2 << mma.getRange());  
    Serial.println("G");
  
}

//Ultrasonic 
long readUltrasonic() {
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);

    long duration = pulseIn(echoPin, HIGH, 20000);
    return duration * 0.034 / 2;
}

//IMU
void sendIMU() {
    float gyroZ = imu.getRotationZ() / 131.0;
    unsigned long now = micros();
    float dt = (now - lastTime) / 1000000.0;
    lastTime = now;
    yaw += gyroZ * dt;
    if (yaw < 0) yaw += 360;
    if (yaw >= 360) yaw -= 360;

    Serial.printf("IMU:%0.2f,0.00,0.00\n", yaw);
}

//Motor Control
void leftMotor(int speed) {
    if (speed >= 0) {
        digitalWrite(AIN1, HIGH);
        digitalWrite(AIN2, LOW);
    } else {
        digitalWrite(AIN1, LOW);
        digitalWrite(AIN2, HIGH);
        speed = -speed;
    }
    ledcWrite(0, speed);
}

void rightMotor(int speed) {
    if (speed >= 0) {
        digitalWrite(BIN1, HIGH);
        digitalWrite(BIN2, LOW);
    } else {
        digitalWrite(BIN1, LOW);
        digitalWrite(BIN2, HIGH);
        speed = -speed;
    }
    ledcWrite(1, speed);
}


void loop() {

    // Jetson commands get handled here
    if (Serial2.available()) {
        char cmd = Serial2.read();

        if (cmd == 'F') { leftMotor(200); rightMotor(200); }
        if (cmd == 'B') { leftMotor(-200); rightMotor(-200); }
        if (cmd == 'L') { leftMotor(-150); rightMotor(150); }
        if (cmd == 'R') { leftMotor(150); rightMotor(-150); }
        if (cmd == 'S') { leftMotor(0); rightMotor(0); }
    }

    // Send ultrasonic
    long distance = readUltrasonic();
    Serial.printf("ULTRASONIC:%ld\n", distance);

    // Send IMU
    sendIMU();

    // Send accelerometer
    mma.read();
    Serial.printf("MMA8451:%0.2f,%0.2f,%0.2f\n", mma.x_g, mma.y_g, mma.z_g);
    
    delay(50);
}