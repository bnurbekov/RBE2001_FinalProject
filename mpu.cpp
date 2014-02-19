#include "rbe2001FinalRobot.h"
#include "MPU6050_6Axis_MotionApps20.h"

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

float getYaw(){
	return ypr[0]*180/M_PI;
}



