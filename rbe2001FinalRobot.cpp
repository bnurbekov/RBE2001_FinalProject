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
			kP = atof(c);
		}
		else if(s.startsWith("ki=")){
			char c[s.substring(3).length()+1];
			s.substring(3).toCharArray(c,sizeof(c));
			kI = atof(c);
		}
		else if(s.startsWith("kd=")){
			char c[s.substring(3).length()+1];
			s.substring(3).toCharArray(c,sizeof(c));
			kD = atof(c);
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
}
