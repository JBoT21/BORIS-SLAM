#include <Arduino.h>
#include <Wire.h>
#include <Servo.h>
#include "esp32-hal-ledc.h"
#include "MPU6050.h"
#include "esp32-hal-gpio.h"
#include <BasicLinearAlgebra.h>
#include <cstdint>

using namespace BLA;

#define SDA_PIN 33
#define SCL_PIN 32

// MPU6050 I2C Address
#define MPU6050_ADDR 0x68  // Default address (0x69 if AD0 pin is high)

/* Define Kalman Parameters per-axis in case they are different*/
#define PITCH_Q 0.001f
#define PITCH_Q_BIAS 0.01f
#define PITCH_R 0.1f
#define ROLL_Q 0.001f
#define ROLL_Q_BIAS 0.003f
#define ROLL_R 0.1f
/* TODO the YAW parameters are almost certainly wrong*/
#define YAW_Q 0.001f
#define YAW_Q_BIAS 0.005f
#define YAW_R 0.03f
/* TODO the VEL parameters are almost certainly wrong*/
#define VEL_Q 0.1f
#define VEL_Q_BIAS 0.01f
#define VEL_R 0.01f

#define K_FORWARD 0.1f
#define K_ROTATION 0.1f
#define TAU 0.2f

enum ValueType
{
    PITCH,
    ROLL,
    YAW,
    VELOCITY
};

typedef struct KalmanFilter
{
    Matrix<2, 1> state; //[angle, bias]
    Matrix<2, 2> P;
    float qValue;
    float qBias;
    float r;
    ValueType type;
} KalmanFilter;

// Global objects
MPU6050 mpu(MPU6050_ADDR, &Wire);


float pitch=0, roll=0, yaw = 0; //gyroscope offsets from start
float x=0, y=0, z=0; //accelerometer calculated position from start
int stationary_counter = 0; // Counter for how many consecutive samples indicate stationary
unsigned long lastTime = 0;

// MPU6050 raw data storage
int16_t ax, ay, az;
int16_t gx, gy, gz;

// MPU 6050 calibrated offsets
long ax_offset = 0, ay_offset = 0, az_offset = 0;
long gx_offset = 0, gy_offset = 0, gz_offset = 0;

/* Kalman Filters */
KalmanFilter pitchKF;
KalmanFilter rollKF;
KalmanFilter yawKF;
KalmanFilter velKF;

/* Kalman Filter Global data */
float x_mps2=0, y_mps2=0, z_mps2=0; //linear accelerations in mps
float leftPWM=0, rightPWM=0; //motor PWM values

/* Data to send to Jetson Nano*/
float forwardVel = 0;
float heading = 0;
float ultrasonicRange = 0;

/* Initialize Kalman Filter Data */
void KalmanInit(KalmanFilter &kf, float initValue, float initBias, float qValue, float qBias, float r, ValueType type)
{
    kf.state(0,0) = initValue;
    kf.state(1,0) = initBias;
    kf.P = Zeros<2,2>();
    kf.qValue = qValue;
    kf.qBias = qBias;
    kf.r = r;
    kf.type = type;
}

/* Update a Kalman Filter given the pre-computed control input */
/* State change and measurement values are computed depending on the ValueType*/
void KalmanUpdate(KalmanFilter &kf, float controlInput, float dt)
{
    /* These matrices are common to all angle measurements*/
    static Matrix<2,2> A = Eye<2,2>();
    static Matrix<2,1> B = Zeros<2,1>();
    static Matrix<1,2> H = {1,0};
    static Matrix<2,2> Q = Zeros<2,2>();
    static Matrix<2,1> K = Zeros<2,1>();
    static Matrix<2,1> xPred = Zeros<2,1>();
    static Matrix<2,2> PPred = Zeros<2,2>();
    static Matrix<2,2> I = Eye<2,2>();
    static float z;
    static float temp;

    /* Instantiate Q*/
    Q(0,0) = kf.qValue;;
    Q(1,1) = kf.qBias;

    /* Take Measurement and Configure State Change and Control Matrices */
    if (kf.type == PITCH)
    {
        A(0,0) = 1;
        A(0, 1) = -dt;
        A(1,0) = 0;
        A(1, 1) = 1;
        z = atan2f(ay, az) * RAD_TO_DEG;
        B(0) = dt;
        // Serial.printf("%0.2f,", accelAngle); //for tuning the R parameter
    }
    else if (kf.type == ROLL)
    {
        A(0,0) = 1;
        A(0, 1) = -dt;
        A(1,0) = 0;
        A(1, 1) = 1;
        z = atan2f(-ax, sqrt(ay*ay + az*az)) * RAD_TO_DEG;
        B(0) = dt;
        // Serial.printf("%0.2f\n", accelAngle); //for tuning the R parameter
    }
    else if (kf.type  == YAW)
    {
        /* Ideally, this would implement first-order lag, as with the forward velocity.
           However, doing so in this case would require a new KF struct and function
           that works with 3x3 matrices. Going to leave this without the lag for now
           until it proves to cause issues with position accuracy
           Turning slowly may mitigate this issue*/
        A(0,0) = 1;
        A(0, 1) = -dt;
        A(1,0) = 0;
        A(1, 1) = 1;
        B(0) = K_ROTATION * dt;
    }
    else if (kf.type == VELOCITY)
    {
        /* dt/TAU terms are a first-order lag adjustment, accounting for the time it takes
           for the motors to spin up when commanded. TAU is the motor response time*/
        A(0,0) = 1 - dt/TAU;
        A(0, 1) = -dt;
        A(1,0) = 0;
        A(1, 1) = 1;
        z = y_mps2*dt + kf.state(0);
        B(0) = K_FORWARD * dt / TAU;
    }
    else
    {
        return;
    }

    /* Prediction Step*/
    xPred = A*kf.state + B*controlInput;
    PPred = A*kf.P*(~A) + Q;

    /*Update Step*/
    temp = (H*PPred*(~H) + kf.r)(0);
    temp = 1.0f/temp;
    K = PPred * (~H) * temp;
    kf.state = xPred + K * (z - H*xPred);
    kf.P = (I - K*H) * PPred;
}

/* Start collecting accelerometer and gyroscope data*/
/* Also perform calibration to remove static biases from the readings*/
void initIMU() {
    int i;
    long sumX=0, sumY=0, sumZ=0, sumGX=0, sumGY=0, sumGZ=0;
    float gravityX=0, gravityY=0, gravityZ=0; //gravity components in mps
    float init_pitch=0, init_roll=0, init_yaw = 0; //gyroscope offsets from start
    float dt;

    Wire.begin(SDA_PIN, SCL_PIN);
    Wire.beginTransmission(MPU6050_ADDR);
    int error = Wire.endTransmission();

    if (error != 0) {
        Serial.printf("Failed to find MPU6050 at address 0x%02X\n", MPU6050_ADDR);
        Serial.println("Checking alternative address 0x69...");

        Wire.beginTransmission(0x69);
        error = Wire.endTransmission();
        if (error == 0) {
            Serial.println("Found MPU6050 at 0x69, updating address");
            // Update the mpu address
            mpu.setSlaveAddress(0,0x69);
        } else {
            Serial.println("MPU6050 not found at either address!");
            while (1);  // Stops here if IMU not found
        }
    }

    mpu.initialize();
    mpu.setDLPFMode(4); //activate 20Hz low pass filter on sensor output

    if (!mpu.testConnection()) {
        Serial.println("MPU6050 connection test failed!");
        while (1);
    }

    // Set up the accelerometer and gyro
    mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_250);  // 250 deg/sec
    mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_2);   // 2g

    /*Calibrate Accelerometer*/
    /*Device must be sitting still during this time*/
    for (i=0; i<200; i++)
    {
        mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
        sumX += ax;
        sumY += ay;
        sumZ += az;
        sumGX += gx;
        sumGY += gy;
        sumGZ += gz;

        /* Remove gravity bias calculations */
        init_pitch = atan2f((float)ay, (float)az) * RAD_TO_DEG;
        init_roll = atan2f(-(float)ax, sqrt((float)ay*(float)ay + (float)az*(float)az)) * RAD_TO_DEG;
        gravityX = -sinf(init_roll * DEG_TO_RAD) * 16384.0f;;
        gravityY = sinf(init_pitch * DEG_TO_RAD) * cosf(init_roll * DEG_TO_RAD) * 16384.0f;
        gravityZ = cosf(init_roll * DEG_TO_RAD) * cosf(init_pitch * DEG_TO_RAD) * 16384.0f;
        sumX -= (long) gravityX;
        sumY -= (long) gravityY;
        sumZ -= (long) gravityZ;
        delay(10);
    }

    ax_offset = sumX / 200;
    ay_offset = sumY / 200;
    az_offset = sumZ / 200;
    gx_offset = sumGX / 200;
    gy_offset = sumGY / 200;
    gz_offset = sumGZ / 200;

    gx_offset = 0;
    gy_offset = 0;

    lastTime = micros();
}


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
    //Serial.printf("Obstacle %ld cm → turning right for %d ms\n", distance, turn_ms);
}

else if (distance < 60) {
    mode = 1;  // turn left
    int turn_ms = map(distance, 20, 40, 700, 400);
    turn_end_time = now + turn_ms;
    heading_index = (heading_index + 3) % 4;
    // Serial.printf("Obstacle %ld cm → turning left for %d ms\n", distance, turn_ms);
}
else {
    // Clear path: go forward
    leftMotor(200);
    rightMotor(200);
    //Serial.printf("Path is clear. Moving forward. Distance: %ld cms.", distance);
}

    // Send data to Jetson for mapping:
    // U: <distance_cm>, H: <heading_index>
    Serial.printf("U:%ld,Y:%0.2f,P:%0.2f,R:%0.2f\n", distance, yaw, pitch, roll);

    delay(50);
}
