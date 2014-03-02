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
double lKP = 0.025;
double lKI = 0.00005;
double lKD = 0;
int setPoint = 0;
float lineVal = 960;
double error, output, integral,derivative,
		lastError, dt,time, lastTime;

double followerL[] = {0,0,0,0,0,0,0}; //dt,error,lasterror,derivative,integral,output
double followerM[] = {0,0,0,0,0,0,1};
double followerR[] = {0,0,0,0,0,0,2};

double l, r, theta, yer, yintegral, ylaster, yderivative,
	youtput;


bool up = false;
bool pollYaw = false;
double upTimer = 0;
double yaw = 0;
double yawOffset = 0;
bool mpuEnabled = true;
double left, right;
int lastRead = -1;

double kPY = 3;
double kIY = 0.5;
double kDY = 0.05;

String s;

//Objects
I2CEncoder encoderArm;
Servo armMotor;
Servo driveL;
Servo driveR;
Servo gripper;

#endif /* VARIABLES_H_ */
