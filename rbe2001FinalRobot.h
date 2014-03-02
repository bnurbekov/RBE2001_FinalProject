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


//Function Declarations
void dmpDataReady();
void code();
void setupMPU6050();
void runMPU6050();
void code();
void timerIsr();
void serialComms();
float getYaw();
void lineFollow(double[]);
void gyroPID();

//Do not add code below this line
#endif /* _rbe2001RobotMega_H_ */
