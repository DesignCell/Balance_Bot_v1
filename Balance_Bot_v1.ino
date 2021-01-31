#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif
MPU6050 mpu;
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high

//Stepper Setup
unsigned int mSpr = 200 * 8; // mSteps Per Rev per uSec :: 200 X microstep
unsigned long pMicrosS1 = 0;
unsigned long pMicrosS2 = 0;
unsigned long pMicrosIntv = 0;
unsigned int intSpdS1;
unsigned int intSpdS2;
int SpdS1t;  //Target Speed
int SpdS2t;  //Target Speed
int SpdS1 = 0;      //Current Speed
int SpdS2 = 0;      //Current Speed
int SpdMax = 60; //Max Speed
int S1r;
int S2r;
int S1c = 1; //Stepper Recalculate Trigger
int S2c = 1; //Stepper Recalculate Trigger
int AngleMPU_X; // MPU Measured Angle In X
int AngleMPU_X_Last; // MPU Last Measure cycle for Differential
int AngleMPU_X_I; // Sum Integral
int AngleMPU_X_D; // Sum Differential
// 100 = 1
int Angle_P = 1000; // Proportional Coefficient
int Angle_I = 1; // Integral Coefficient
int Angle_D = 0; // Differential Coefficient
int SpdMPU_P = 0; // Proportional Product
int SpdMPU_I = 0; // Integral Product
int SpdMPU_D = 0; // Differential Prodcut

void dmpDataReady() {
	mpuInterrupt = true;
}

// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

void setup() {
	// join I2C bus (I2Cdev library doesn't do this automatically)
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
	Wire.begin();
	TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz). Comment this line if having compilation difficulties with TWBR.
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
	Fastwire::setup(400, true);
#endif
	Serial.begin(115200);
	while (!Serial); // wait for Leonardo enumeration, others continue immediately
	mpu.initialize();
	devStatus = mpu.dmpInitialize();

	// supply your own gyro offsets here, scaled for min sensitivity
	mpu.setXGyroOffset(220);
	mpu.setYGyroOffset(76);
	mpu.setZGyroOffset(-85);
	mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

							   // make sure it worked (returns 0 if so)
	if (devStatus == 0) {
		// turn on the DMP, now that it's ready
		Serial.println(F("Enabling DMP..."));
		mpu.setDMPEnabled(true);

		// enable Arduino interrupt detection
		Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
		attachInterrupt(0, dmpDataReady, RISING);
		mpuIntStatus = mpu.getIntStatus();

		// set our DMP Ready flag so the main loop() function knows it's okay to use it
		Serial.println(F("DMP ready! Waiting for first interrupt..."));
		dmpReady = true;

		// get expected DMP packet size for later comparison
		packetSize = mpu.dmpGetFIFOPacketSize();
	}
	else {
		// ERROR!
		// 1 = initial memory load failed
		// 2 = DMP configuration updates failed
		// (if it's going to break, usually the code will be 1)
		Serial.print(F("DMP Initialization failed (code "));
		Serial.print(devStatus);
		Serial.println(F(")"));
	}

	//Stepper Setup
	pinMode(3, OUTPUT);digitalWrite(3, LOW);
	pinMode(4, OUTPUT);digitalWrite(4, HIGH); //Default CW
	pinMode(5, OUTPUT);digitalWrite(5, LOW);
	pinMode(6, OUTPUT);digitalWrite(6, HIGH); //Default CW

}

void loop() {
	// if programming failed, don't try to do anything
	if (!dmpReady) return;

	// wait for MPU interrupt or extra packet(s) available
	while (!mpuInterrupt && fifoCount < packetSize) {



		//System Clock ms time
		///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		unsigned long cMicros = micros();

		//Stepper #1
		//One-Shot Stepper Recalculate Trigger
		//RPM Ramp +/-1
		//Skip if 0 RPM
		//Sense of Roration :: CCW(+) CW(-)
		///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		if (S1c == 1 || SpdS1 == 0) {
			S1c = 0; //Stepper Recalculate Trigger Reset
			if ((SpdS1t > SpdS1) && (SpdS1t != SpdS1)) SpdS1 = SpdS1 + 1; //If target greater than current && not =, ++
			if ((SpdS1t < SpdS1) && (SpdS1t != SpdS1)) SpdS1 = SpdS1 - 1; //If target less than current && not =, --
			if (SpdS1 != 0) {
				if (SpdS1 >0) {
					S1r = 1;
					pinMode(4, OUTPUT);digitalWrite(4, LOW); //CCW
				}
				else {
					S1r = -1;
					pinMode(4, OUTPUT);digitalWrite(4, HIGH); // CW
				}
				intSpdS1 = (long int)1000000 / (abs(SpdS1)*(long int)mSpr / 60);
			}
			else S1r = 0;
		}

		//Stepper #2
		//One-Shot Stepper Recalculate Trigger
		//RPM Ramp +/-1
		//Skip if 0 RPM
		//Sense of Roration :: CCW(+) CW(-)
		///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		if (S2c == 1 || SpdS2 == 0) {
			S2c = 0; //Stepper Recalculate Trigger Reset
			if ((SpdS2t > SpdS2) && (SpdS2t != SpdS2)) SpdS2 = SpdS2 + 1; //If target greater than current && not =, ++
			if ((SpdS2t < SpdS2) && (SpdS2t != SpdS2)) SpdS2 = SpdS2 - 1; //If target less than current && not =, --
			if (SpdS2 != 0) {
				if (SpdS2 >0) {
					S2r = 1;
					pinMode(6, OUTPUT);digitalWrite(6, LOW); // CCW
				}
				else {
					S2r = -1;
					pinMode(6, OUTPUT);digitalWrite(6, HIGH); // CW
				}
				intSpdS2 = (long int)1000000 / (abs(SpdS2)*(long int)mSpr / 60);
			}
			else S2r = 0;
		}

		//Step Stepper
		///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		if ((cMicros - pMicrosS1 > intSpdS1) && (abs(S1r) == 1)) {
			pMicrosS1 = cMicros;
			pinMode(3, OUTPUT);digitalWrite(3, HIGH);
			S1c = 1; //Stepper Recalculate Trigger
		}

		if ((cMicros - pMicrosS2 > intSpdS2) && (abs(S2r) == 1)) {
			pMicrosS2 = cMicros;
			pinMode(5, OUTPUT);digitalWrite(5, HIGH);
			S2c = 1; //Stepper Recalculate Trigger
		}

		delayMicroseconds(1);
		pinMode(3, OUTPUT);digitalWrite(3, LOW);
		pinMode(5, OUTPUT);digitalWrite(5, LOW);


	}

	mpuInterrupt = false;
	mpuIntStatus = mpu.getIntStatus();
	fifoCount = mpu.getFIFOCount();
	if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
		mpu.resetFIFO();
		Serial.println(F("FIFO overflow!"));
	}
	else if (mpuIntStatus & 0x02) {
		while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
		mpu.getFIFOBytes(fifoBuffer, packetSize);
		fifoCount -= packetSize;

		// display Euler angles in degrees
		mpu.dmpGetQuaternion(&q, fifoBuffer);
		mpu.dmpGetGravity(&gravity, &q);
		mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

		//MPU Angle
		AngleMPU_X = ypr[2] * 180 / M_PI; //yrp[0] yaw //ypr[1] roll //ypr[2] pitch

		AngleMPU_X_I = AngleMPU_X_I + AngleMPU_X; //Sum Integral
		AngleMPU_X_D = AngleMPU_X - AngleMPU_X_Last; //Differential
		AngleMPU_X_Last = AngleMPU_X; //Set last angle

		//Stepper Control during While
		SpdMPU_P = AngleMPU_X*Angle_P / 100;
		SpdMPU_I = AngleMPU_X_I*Angle_I / 100;
		SpdMPU_D = AngleMPU_X_D*Angle_D / 100;

		SpdS1t = (SpdMPU_P + SpdMPU_I + SpdMPU_D);
		SpdS2t = -(SpdMPU_P + SpdMPU_I + SpdMPU_D);

		// Set Speed Limit
		if (SpdS1t < -300) SpdS1t = -300;
		if (SpdS2t < -300) SpdS2t = -300;
		if (SpdS1t > 300) SpdS1t = 300;
		if (SpdS2t > 300) SpdS2t = 300;

		// Set Speed to Zero When Fallen
		if (AngleMPU_X < -45 || AngleMPU_X > 45) {
			SpdS1t = 0;
			SpdS2t = 0;
		}

		Serial.println(SpdS1t);



	}
}
