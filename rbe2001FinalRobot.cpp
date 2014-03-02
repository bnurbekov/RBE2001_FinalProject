#include "RBE2001FinalRobot.h"
#include "Variables.h"

void setup()
{
	Wire.begin();
	encoderArm.init(1, MOTOR_393_TIME_DELTA);
	encoderArm.zero();
	armMotor.attach(3,1000,2000);
	driveL.attach(4,1000,2000);
	driveR.attach(5,1000,2000);
	gripper.attach(19,600,2400);
	time = micros();
	upTimer = time;
	Serial.begin(115200);

	if(mpuEnabled)
		setupMPU6050();
}

void loop()
{
	if(mpuEnabled)
		runMPU6050();
	else
		code();
}

void code(){
	lastTime = time;
	time = micros();
	dt = (time-lastTime)/1000000;

	followerL[0] = dt;
	followerM[0] = dt;
	followerR[0] = dt;

	lineFollow(followerL);
	lineFollow(followerM);
	lineFollow(followerR);

	gyroPID();

	//if(followerM[1] <= 700 || followerL[1] <= 700 || followerR[1] <= 700){
		driveL.write(left);		//110 + followerL[5]-followerR[5]);
		driveR.write(right);		//70 - followerR[5]+followerL[5]);
	//}
	//else{
		//driveL.write(90);
		//driveR.write(90);
	//}

	serialComms();

	yaw = getYaw() - yawOffset;

	lastError = error;
	error = setPoint - encoderArm.getPosition();

	if(dt != 0)
		derivative = (error-lastError)/dt;
	else
		derivative = 0;

	integral += error*dt;

	output = error*kP + integral*kI + derivative*kD;

	if(abs(output) > 90)
		output = 90*(abs(output)/output);

	if(output < -30)
		armMotor.write((-30)+90);
	else
		armMotor.write(output+90);

}

void serialComms(){
	if (Serial.available()) {
		s = Serial.readStringUntil('\n');
		if(s.equals("zero")){
			encoderArm.zero();
			integral = 0;
			derivative = 0;
			setPoint = 0;
		}
		else if(s.equals("pollYaw") && mpuEnabled)
			pollYaw ^= 1;
		else if(s.equals("zeroYaw") && mpuEnabled)
			yawOffset = getYaw();
		else if(s.equals("rawYaw"))
			yawOffset = 0;
		else if(s.startsWith("kp=")){
			char c[s.substring(3).length()+1];
			s.substring(3).toCharArray(c,sizeof(c));
			lKP = atof(c);
		}
		else if(s.startsWith("ki=")){
			char c[s.substring(3).length()+1];
			s.substring(3).toCharArray(c,sizeof(c));
			lKI = atof(c);
		}
		else if(s.startsWith("kd=")){
			char c[s.substring(3).length()+1];
			s.substring(3).toCharArray(c,sizeof(c));
			lKD = atof(c);
		}
		else if(s.startsWith("gripper")){
			char c[s.substring(7).length()+1];
			s.substring(7).toCharArray(c,sizeof(c));
			gripper.write(constrain(atoi(c),0,180));
		}
		else{
			int n = s.toInt();
			setPoint = n;
		}
		Serial.read();
	}
	if(pollYaw && mpuEnabled)
		Serial.println(yaw);
	//Serial.print(followerL[5]);
	//Serial.print("\t");
	//Serial.println(followerM[1]);
}

void lineFollow(double lFollower[]){
	switch((int)lFollower[6]){
	case 0://left follower
			followerL[2] = followerL[1];
			followerL[1] = lineVal - analogRead(13);

			if(followerL[0] != 0)
				followerL[3] = (followerL[1]-followerL[2])/followerL[0];
			else
				followerL[3] = 0;

			followerL[4] += followerL[1]*followerL[0];

			followerL[5] = followerL[1]*lKP + followerL[4]*lKI + followerL[3]*lKD;

			if(abs(followerL[5]) > 90)
				followerL[5] = 90*(abs(followerL[5])/followerL[5]);
			break;
		break;
	case 1://mid follower
		followerM[2] = followerM[1];
		followerM[1] = lineVal - analogRead(14);

		if(followerM[0] != 0)
			followerM[3] = (followerM[1]-followerM[2])/followerM[0];
		else
			followerM[3] = 0;

		followerM[4] += followerM[1]*followerM[0];

		followerM[5] = followerM[1]*lKP + followerM[4]*lKI + followerM[3]*lKD;

		if(abs(followerM[5]) > 90)
			followerM[5] = 90*(abs(followerM[5])/followerM[5]);
		break;
	case 2://right follower
		followerR[2] = followerR[1];
		followerR[1] = lineVal - analogRead(15);

		if(followerR[0] != 0)
			followerR[3] = (followerR[1]-followerR[2])/followerR[0];
		else
			followerR[3] = 0;

		followerR[4] += followerR[1]*followerR[0];

		followerR[5] = followerR[1]*lKP + followerR[4]*lKI + followerR[3]*lKD;

		if(abs(followerR[5]) > 90)
			followerR[5] = 90*(abs(followerR[5])/followerR[5]);
		break;
	}
}

void gyroPID(){
	double kP = 2;
	double kI = 2;
	double kD = 0.03;

	theta = 0;

	if(abs(yer) < 20)
		yintegral += yer*dt;
	else
		yintegral = 0;

	if(yaw - theta > 180)
		theta = 360 - theta;

	if(theta - yaw > 180)
		theta = theta - 360;

	ylaster = yer;
	yer = theta - yaw;
	if(dt != 0)
		yderivative = (yer-ylaster)/dt;

	else
		yderivative = 0;

	youtput = yer*kP + yderivative*kD + yintegral*kI;

	left = 90 + youtput;
	right = 90 + youtput;

	if(left > 150)
		left = 150;
	else if(left <  30)
		left = 30;

	if(right > 150)
		right = 150;
	else if(right <  30)
		right = 30;
}

