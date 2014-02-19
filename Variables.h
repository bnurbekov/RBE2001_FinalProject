/*
 * variables.h
 *
 *  Created on: Feb 14, 2014
 *      Author: Jason
 */

#ifndef VARIABLES_H_
#define VARIABLES_H_

//Variable Declarations

//PID
double kP = 7;
double kI = 3;
double kD = 0.4;
int setPoint = 0;
double error, output, integral,derivative,
		lastError, dt,time, lastTime;

bool up = false;
bool pollYaw = false;
double upTimer = 0;
double yaw = 0;
double yawOffset = 0;
bool mpuEnabled = true;
String s;

//Objects
I2CEncoder encoderArm;
Servo armMotor;
Servo driveL;
Servo driveR;
Servo gripper;

#endif /* VARIABLES_H_ */
