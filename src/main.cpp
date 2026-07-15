#include <Arduino.h>
#include <Wire.h>
#include "esp32-hal-ledc.h"
#include "esp32-hal-gpio.h"
#include <cstdint>
#include "MPU6050.h"

MPU6050 mpu;

// Ultrasonic pins
const int trigPin = 4;
const int echoPin = 15;

// TB6612 Motor Driver Pins
#define PWMA 5 
#define AIN1 16
#define AIN2 17 
#define PWMB 23
#define BIN1 25 //was 21
#define BIN2 26 //Was 22

// Heading index: 0=North, 1=East, 2=South, 3=West
int heading_index = 0;

// Motor control
float leftPWM = 0.0f;
float rightPWM = 0.0f;

long readUltrasonic() {
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);

    long duration = pulseIn(echoPin, HIGH, 60000);
    if (duration == 0) {
        return 803; 
    }
    return duration * 0.034 / 2;  // cm
}

void leftMotor(int speed) {
    speed = constrain(speed, -255, 255);
    leftPWM = (float)speed;

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
    speed = constrain(speed, -255, 255);
    rightPWM = (float)speed;

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

void stopMotors() {
    leftMotor(0);
    rightMotor(0);
    leftPWM = 0.0f;
    rightPWM = 0.0f;
}

void setup() {
    Serial.begin(115200);

    mpu.initialize();
    if (!mpu.testConnection()) {
        Serial.println("MPU6050 connection failed!");
    } else {
        Serial.println("MPU6050 connected.");
    }

    // Ultrasonic
    pinMode(trigPin, OUTPUT);
    pinMode(echoPin, INPUT);

    // Motor pins
    pinMode(AIN1, OUTPUT);
    pinMode(AIN2, OUTPUT);
    pinMode(BIN1, OUTPUT);
    pinMode(BIN2, OUTPUT);

    // PWM channels
    ledcSetup(0, 1000, 8);
    ledcAttachPin(PWMA, 0);
    ledcSetup(1, 1000, 8);
    ledcAttachPin(PWMB, 1);

    stopMotors();
}

void loop() {

    int16_t ax, ay, az;
    int16_t gx, gy, gz;

    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    // Convert raw gyro to degrees/sec
    float gyroX = gx / 131.0;
    float gyroY = gy / 131.0;
    float gyroZ = gz / 131.0;

    // Simple integration for yaw/pitch/roll (not perfect but works)
    static float yaw = 0, pitch = 0, roll = 0;
    float dt = 0.05;  // 50ms loop

    yaw   += gyroZ * dt;
    pitch += gyroY * dt;
    roll  += gyroX * dt;

    // Handle explicit commands from Jetson (optional override)
    if (Serial.available()) {
        char cmd = Serial.read();

        if (cmd == 'F') {
            leftMotor(200);
            rightMotor(200);
        }
        else if (cmd == 'B') {
            leftMotor(-200);
            rightMotor(-200);
        }
        else if (cmd == 'L') {
            // Turn left in place
            leftMotor(-150);
            rightMotor(150);
            heading_index = (heading_index + 3) % 4;  // rotate left
        }
        else if (cmd == 'R') {
            // Turn right in place
            leftMotor(150);
            rightMotor(-150);
            heading_index = (heading_index + 1) % 4;  // rotate right
        }
        else if (cmd == 'S') {
            stopMotors();
        }
    }

    // Reactive behavior based solely on ultrasonic distance
    long distance = readUltrasonic();

    // Simple obstacle-avoidance logic:
    // - very close obstacle (<20 cm): stop and turn right
    // - close obstacle (<40 cm): hard left turn
    // - Otherwise: forward
    
static unsigned long turn_end_time = 0;
static int mode = 0;  
// 0 = forward
// 1 = turn left
// 2 = turn right

unsigned long now = millis();

// If currently turning, continue until timer expires
if (now < turn_end_time) {
    if (mode == 1) {
        leftMotor(-150);
        rightMotor(150);
    } else if (mode == 2) {
        leftMotor(150);
        rightMotor(-150);
    }
}


if (distance < 20) {
    mode = 2;  // turn right
    int turn_ms = map(distance, 0, 20, 900, 500);
    turn_end_time = now + turn_ms;
    heading_index = (heading_index + 1) % 4;
    Serial.printf("Obstacle %ld cm → turning right for %d ms\n", distance, turn_ms);
}

else if (distance < 60) {
    mode = 1;  // turn left
    int turn_ms = map(distance, 20, 40, 700, 400);
    turn_end_time = now + turn_ms;
    heading_index = (heading_index + 3) % 4;
    Serial.printf("Obstacle %ld cm → turning left for %d ms\n", distance, turn_ms);
}
else {
    // Clear path: go forward
    leftMotor(200);
    rightMotor(200);
    Serial.printf("Path is clear. Moving forward. Distance: %ld cms.", distance);
}

    // Send data to Jetson for mapping:
    // U: <distance_cm>, H: <heading_index>
    Serial.printf("U:%ld,Y:%0.2f,P:%0.2f,R:%0.2f\n", distance, yaw, pitch, roll);

    delay(50);
}
