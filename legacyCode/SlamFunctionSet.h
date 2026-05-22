/*
 * SLAMFunctionSet.h
 * 
 * SLAM Integration for ELEGOO Conqueror Robot Tank
 * Note: This was adapted from: https://github.com/PatelVatsalB21/Ultrasonic-SLAM
 * This header defines the SLAMFunctionSet class, which encapsulates all SLAM-related functionality for the Conqueror robot.
 */
 
#ifndef _SLAMFUNCTIONSET_xxx0_H_
#define _SLAMFUNCTIONSET_xxx0_H_
 
#include <Arduino.h>
#include <Servo.h>
 
// Forward declaration to avoid circular dependency
class DeviceDriverSet_Motor_Class;
 
class SLAMFunctionSet {
public:
    //SLAM Status
    bool slamMappingActive;
    bool slamNavigationActive;
    bool slamInitialized;
    
    //Map Storage 
    static const int MAX_MAP_SIZE = 100;
    int readings[MAX_MAP_SIZE];
    byte mapsize;
    int currentIteration;
    
    // Timing Data
    unsigned long prev_t;
    unsigned long curr_t;
    
    // Calibration Parameters 
    // If modiefied for any other robot, this WILL need to be changed!
    // unit_TH: Time (microseconds) for robot to travel one robot-length horizontally
    // unit_TV: Time (microseconds) for robot to travel one robot-width vertically
    unsigned long unit_TH;  
    unsigned long unit_TV;
    
    // Current Reading Variables 
    byte l;  // Left side reading
    byte b;  // Base/forward reading
    byte r;  // Right side reading
    byte base_size;
    
    // Hardware Configuration
    byte ultrasonicTrigPin;
    byte ultrasonicEchoPin;
    byte slamServoPin;
    Servo slamServo;
    
    // Public Methods
    
    // Initialization
    void initializeSLAM();
    void calibrateSLAM();
    
    // Main SLAM execution functions
    void executeMapping();
    void executeNavigation();
    void stopSLAM();
    
    // Movement wrapper functions (interface to Conqueror's motor system)
    void slam_moveForward();
    void slam_moveForwardContinuous();
    void slam_moveStop();
    void slam_moveOneBlock();
    void slam_turnRight();
    void slam_turnLeft();
    
    // Sensor functions
    int slam_readUltrasonic();
    int slam_lookForward();
    int slam_lookRight();
    int slam_lookLeft();
    
    // Map management functions
    void slam_addReading(int iteration);
    void slam_expandReadingsArray();
    void slam_printMap();
    void slam_saveMap();
    void slam_loadMap();
    void slam_clearMap();
    
    // Utility functions
    long slam_microsecondsToCentimeters(long microseconds);
    void slam_setCalibration(unsigned long horizontal_time, unsigned long vertical_time);
    
private:
    // Helper functions
    void slam_mappingIteration();
    bool slam_checkObstacle();
};


 
#endif 
// Global instance declaration
extern SLAMFunctionSet SLAM_FunctionSet;