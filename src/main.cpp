#include <Arduino.h>
#include <Wire.h>
#include <Servo.h>
#include "MPU6050.h"

// Hardware Pin Definitions
const int trigPin = 5;
const int echoPin = 18;

const int motorLeftA  = 26;
const int motorLeftB  = 27;
const int motorRightA = 32;
const int motorRightB = 33;

const int servoPin = 14;

//Objects

Servo scanServo;
MPU6050 imu;

