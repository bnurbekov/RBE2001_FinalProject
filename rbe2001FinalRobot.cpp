#include "RBE2001FinalRobot.h"

void setup()
{
	Wire.begin();
	encoderArm.init(1, MOTOR_393_TIME_DELTA);
	encoderArm.zero();
	armMotor.attach(3,1000,2000);
	setupMPU6050();
	time = micros();
	upTimer = time;
	Serial.begin(115200);
}

void loop()
{
	runMPU6050();
}

void dmpDataReady() {
    mpuInterrupt = true;
}

void setupMPU6050(){
	// join I2C bus (I2Cdev library doesn't do this automatically)
	#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
		Wire.begin();
		TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
	#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
		Fastwire::setup(400, true);
	#endif

	mpu.initialize();
	devStatus = mpu.dmpInitialize();

	mpu.setXGyroOffset(135);
	mpu.setYGyroOffset(21);
	mpu.setZGyroOffset(6.75);
	mpu.setXAccelOffset(-312);
	mpu.setYAccelOffset(-1223);
	mpu.setZAccelOffset(1535);

	// make sure it worked (returns 0 if so)
	if(devStatus == 0) {
	  // turn on the DMP, now that it's ready
	  mpu.setDMPEnabled(true);

	  // enable Arduino interrupt detection
	  attachInterrupt(0, dmpDataReady, RISING);
	  mpuIntStatus = mpu.getIntStatus();

	  // set our DMP Ready flag so the main loop() function knows it's okay to use it
	  dmpReady = true;

	  // get expected DMP packet size for later comparison
	  packetSize = mpu.dmpGetFIFOPacketSize();
	}
}

void runMPU6050(){
	if(!dmpReady) return;

	// reset interrupt flag and get INT_STATUS byte
	mpuInterrupt = false;
	mpuIntStatus = mpu.getIntStatus();

	// get current FIFO count
	fifoCount = mpu.getFIFOCount();

	// check for overflow (this should never happen unless our code is too inefficient)
	if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
	  // reset so we can continue cleanly
	  mpu.resetFIFO();

	// otherwise, check for DMP data ready interrupt (this should happen frequently)
	}

	else if (mpuIntStatus & 0x02) {
	// wait for correct available data length, should be a VERY short wait
	while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

	// read a packet from FIFO
	mpu.getFIFOBytes(fifoBuffer, packetSize);

	// track FIFO count here in case there is > 1 packet available
	// (this lets us immediately read more without waiting for an interrupt)
	fifoCount -= packetSize;

	mpu.dmpGetQuaternion(&q, fifoBuffer);
	mpu.dmpGetGravity(&gravity, &q);
	mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
	}

	while(!mpuInterrupt && fifoCount < packetSize){
	  code();
	}
}

double kP = 7;
double kI = 3;
double kD = 0.4;

void code(){
	autoLift(45);
	lastTime = time;
	time = micros();
	dt = (time-lastTime)/1000000;

	yaw = (ypr[0]*180/M_PI) - yawOffset;

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
		else if(s.equals("pollYaw"))
			pollYaw ^= 1;
		else if(s.equals("zeroYaw"))
			yawOffset = ypr[0]*180/M_PI;
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

		else{
			int n = s.toInt();
			setPoint = n;
		}
		Serial.read();
	}

	if(pollYaw)
		Serial.println(yaw);
}

void autoLift(int height){
	if(up)
		setPoint = height;
	else
		setPoint = 0;

	if(time-upTimer >= 5000000){
		up ^= 1;
		upTimer = time;
	}
}
