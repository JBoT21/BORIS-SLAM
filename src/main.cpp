#include <Arduino.h>
#include <Wire.h>
#include <Servo.h>
#include "esp32-hal-ledc.h"
#include "MPU6050.h"
#include "esp32-hal-gpio.h"
#include <BasicLinearAlgebra.h>
#include <cstdint>

using namespace BLA;

#define NUM_STATIONARY_SAMPLES 5
#define ACC_THRESHOLD 0.1  // Threshold in mps^2 to consider the robot as moving or stationary
#define GYRO_THRESHOLD 3   // Threshold in deg/sec to consider the robot as moving or stationary
#define G 9.8067f //gravitational accelration in m/s^2

// Hardware Pin Definitions (Adjust ESP32 pins as needed)
// Note: GPIO 6-11 are reserved for flash memory on most ESP32 boards
// (So don't use those)

// Pin definitions
const int trigPin = 4;
const int echoPin = 15;

// TB6612 Motor Driver Pins
#define PWMA 5 
#define AIN1 19
#define AIN2 18 
#define PWMB 23
#define BIN1 21
#define BIN2 22
//#define STBY 2

//MMA8451 Accelerometer Pins (I2C)
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
float heading = 20;
float ultrasonicRange = 0;
float heading_slam = 0;


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

void readIMU() {
    static float gravityX, gravityY, gravityZ; //gravity components in mps
    
    float xOff, yOff, zOff; //linear offsets from last position.
    float gyroX_dps, gyroY_dps, gyroZ_dps;
    float dt;
    float acc_magnitude;
    float gyro_magnitude;
    int i;
    float forwardPWM;
    float rotationPWM;

    /* Timekeeping */
    unsigned long now = micros();
    dt = (now - lastTime) / 1e6f;  // Converts to seconds
    lastTime = now;

    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    /* Apply calibrated offsets to remove per-axis bias*/
    ax -= ax_offset;
    ay -= ay_offset;
    az -= az_offset;
    gx -= gx_offset;
    gy -= gy_offset;
    gz -= gz_offset;
    
    /* Convert 3-axis acceleration to m/s^2*/
    x_mps2 = (ax / 16384.0f * G);
    y_mps2 = (ay / 16384.0f * G);
    z_mps2 = (az / 16384.0f * G);

    // Convert raw gyro values to degrees per second (250 deg/sec range)
    gyroX_dps = gx / 131.0f;
    gyroY_dps = gy / 131.0f;
    gyroZ_dps = gz / 131.0f;

    /* Compute total acceleration magnitude in m/s^2*/
    acc_magnitude = sqrt(x_mps2*x_mps2 + y_mps2*y_mps2 + z_mps2*z_mps2) - G; // Subtract gravity to get net acceleration
    gyro_magnitude = sqrt(gyroX_dps*gyroX_dps + gyroY_dps*gyroY_dps + gyroZ_dps*gyroZ_dps);

    /* Detect stationary state */
    if (gyro_magnitude < GYRO_THRESHOLD && acc_magnitude < ACC_THRESHOLD)
        stationary_counter++;
    else
        stationary_counter = 0; // Reset counter if we detect movement
    
    /*Set all velocities to zero if stationary*/
    /*Must be done before Kalman update to filter out gyro noise*/
    if (stationary_counter >= NUM_STATIONARY_SAMPLES)
    {
        gyroX_dps = 0;
        gyroY_dps = 0;
        gyroZ_dps = 0;
        forwardVel = 0;
    }

    KalmanUpdate(pitchKF, gyroX_dps, dt);
    KalmanUpdate(rollKF, gyroY_dps, dt);
    pitch = pitchKF.state(0);
    roll = rollKF.state(0);

    /* Use the Kalman-adjusted orientation to determine gravity if moving */
    if (stationary_counter < NUM_STATIONARY_SAMPLES)
    {
        gravityX = -sinf(roll * DEG_TO_RAD) * G;
        gravityY = sinf(pitch * DEG_TO_RAD) * cosf(roll * DEG_TO_RAD) * G;
        gravityZ = cosf(roll * DEG_TO_RAD) * cosf(pitch * DEG_TO_RAD) * G;
        x_mps2 -= gravityX;
        y_mps2 -= gravityY;
        z_mps2 -= gravityZ;
    }

    /* Compute Forward (y-axis) velocity*/
    forwardPWM = (leftPWM+rightPWM) / 2.0;
    KalmanUpdate(velKF, forwardPWM, dt);
    forwardVel = velKF.state(0);

    /* Compute bearing (relative to starting orientation)*/
    rotationPWM = (leftPWM - rightPWM);
    KalmanUpdate(yawKF, gyroZ_dps, dt); //Was rotationPWM, but that was causing yaw errors. Using gyroZ_dps instead
    heading = yawKF.state(0);

    /* theta measures bearing in degrees RIGHT of the y axis */
    /* Normalize to (-180, 180]*/
    while (heading > 180) heading -= 360; 
    while (heading <= -180) heading += 360;



    /* Forward velocity and heading will be sent in main loop*/
    /* Final pose calculations will be computed by the Jetson Nano*/
}

// Ultrasonic distance reading
long readUltrasonic() {
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);

    long duration = pulseIn(echoPin, HIGH, 60000);
    return duration * 0.034 / 2;  // Convert to cm
}

// Motor Control Functions
void leftMotor(int speed) {
    // Clamp speed to valid range
    speed = constrain(speed, -255, 255);
    leftPWM = (float) speed; //preserve the sign for KF usage
    
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
    // Clamp speed to valid range
    speed = constrain(speed, -255, 255);
    rightPWM = (float) speed; //preserve the sign for KF usage
    
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

// Stop all motors
void stopMotors() {
    leftMotor(0);
    rightMotor(0);
    leftPWM = 0.0f;
    rightPWM = 0.0f;
}

void setup() {
    Serial.begin(115200);  // Used to communicate w/ computer (Sensor data output)

    // Ultrasonic
    pinMode(trigPin, OUTPUT);
    pinMode(echoPin, INPUT);
    // Serial.println("Ultrasonic Initialized");

    // Motor pins
    pinMode(AIN1, OUTPUT);
    pinMode(AIN2, OUTPUT);
    pinMode(BIN1, OUTPUT);
    pinMode(BIN2, OUTPUT);
    //pinMode(STBY, OUTPUT);
    //digitalWrite(STBY, HIGH);
    
    // PWM channels for motors
    ledcSetup(0, 1000, 8);
    ledcAttachPin(PWMA, 0);
    ledcSetup(1, 1000, 8);
    ledcAttachPin(PWMB, 1);
    // Serial.println("Motors Initialized");

    /* Kalman Filters */
    KalmanInit(pitchKF, 0, 0, PITCH_Q, PITCH_Q_BIAS, PITCH_R, PITCH);
    KalmanInit(rollKF, 0, 0, ROLL_Q, ROLL_Q_BIAS, ROLL_R, ROLL);
    KalmanInit(yawKF, 0, 0, YAW_Q, YAW_Q_BIAS, YAW_R, YAW);
    KalmanInit(velKF, 0, 0, VEL_Q, VEL_Q_BIAS, VEL_R, VELOCITY);

    // IMU
    initIMU();
}

void loop() {
    // Jetson commands handling
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
            leftMotor(-150);
            rightMotor(150);
        }
        else if (cmd == 'R') {
            leftMotor(150);
            rightMotor(-150);
        }
        else if (cmd == 'S') {
            stopMotors();
        }
    }

    static uint32_t now = 0;

    now = millis();

    // Read ultrasonic data
    long distance = readUltrasonic();
    //Serial.printf("ULTRASONIC: %ld cm\n", distance);

    // Send IMU (gyro-based yaw)
    readIMU();

    /* Send data to Jetson nano for position determination and SLAM map building*/
    /* 
        Jetson needs the following data:
        1. Forward Velocity
        2. Heading
        3. Ultrasonic range
    */
    //Serial.printf("IMU: %0.4f,%0.4f,%ld\n", forwardVel,heading,distance);

    //Serial.printf("IMU: %0.4f,%0.4f,%0.4f\n", pitchKF.state(0), rollKF.state(0), heading);

    Serial.printf("U:%ld,Y:%.2f,P:%.2f,R:%.2f\n",
              distance, heading, pitchKF.state(0), rollKF.state(0));
    
    delay(50);
}