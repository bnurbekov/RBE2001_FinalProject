// Only modify this file to include
// - function definitions (prototypes)
// - include files
// - extern variable definitions
// In the appropriate section

#ifndef _rbe2001FinalRobot_H_
#define _rbe2001FinalRobot_H_
#include "Arduino.h"
//add your includes for the project rbe2001RobotTest here
#include "Wire.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "I2CEncoder.h"
#include "Servo.h"

//end of add your includes here
#ifdef __cplusplus
extern "C" {
#endif
void loop();
void setup();
#ifdef __cplusplus
} // extern "C"
#endif

//add your function definitions for the project rbe2001RobotTest here

MPU6050 mpu;
volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

//Variable Declarations
int setPoint = 0;
bool up = false;
bool pollYaw = false;
double upTimer = 0;
double error, output, integral,derivative,lastError,dt,time,lastTime;
double yaw = 0;
double yawOffset = 0;
String s;

//Objects
I2CEncoder encoderArm;
Servo armMotor;

//Function Declarations
void dmpDataReady();
void code();
void setupMPU6050();
void runMPU6050();
void code();
void timerIsr();
void serialComms();
void autoLift(int);

//Do not add code below this line
#endif /* _rbe2001RobotMega_H_ */
