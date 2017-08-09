/*

IMU-Sine-Synth Example from Bela On Ur Head

by Becky Stewart 2017

See the bottom of this file for more details on how to use it.


This code is based on:
MrHeadTracker https://git.iem.at/DIY/MrHeadTracker
Adafruit BNO055 Sensor library https://github.com/adafruit/Adafruit_BNO055


This software is distributed under the GNU Lesser General Public License
(LGPL 3.0), available here: https://www.gnu.org/licenses/lgpl-3.0.txt

---------------------------------------------------------------
 ____  _____ _        _    
| __ )| ____| |      / \   
|  _ \|  _| | |     / _ \  
| |_) | |___| |___ / ___ \ 
|____/|_____|_____/_/   \_\

The platform for ultra-low latency audio and sensor processing

http://bela.io

A project of the Augmented Instruments Laboratory within the
Centre for Digital Music at Queen Mary University of London.
http://www.eecs.qmul.ac.uk/~andrewm

(c) 2016 Augmented Instruments Laboratory: Andrew McPherson,
  Astrid Bin, Liam Donovan, Christian Heinrichs, Robert Jack,
  Giulio Moro, Laurel Pardue, Victor Zappi. All rights reserved.

The Bela software is distributed under the GNU Lesser General Public License
(LGPL 3.0), available here: https://www.gnu.org/licenses/lgpl-3.0.txt
*/


#include <Bela.h>
#include <cmath>
#include <rtdk.h>
#include "Bela_BNO055.h"


// Change this to change how often the BNO055 IMU is read (in Hz)
int readInterval = 10;

I2C_BNO055 bno; // IMU sensor object
int buttonPin = P8_08; // calibration button pin
int lastButtonValue = 0; // using a pulldown resistor

// Quaternions and Vectors
imu::Quaternion gCal, gCalLeft, gCalRight, gIdleConj = {1, 0, 0, 0};
imu::Quaternion qGravIdle, qGravCal, quat, steering, qRaw;

imu::Vector<3> gRaw;         
imu::Vector<3> gGravIdle, gGravCal;
imu::Vector<3> ypr; //yaw pitch and roll angles

// variables for synth
float gPhase = 0; 
float gFrequency = 400; // frequency of sine wave
float gInverseSampleRate; 

int calibrationState = 0; // state machine variable for calibration
int setForward = 0; // flag for setting forward orientation

// variables handling threading
AuxiliaryTask i2cTask;		// Auxiliary task to read I2C
AuxiliaryTask gravityNeutralTask;		// Auxiliary task to read gravity from I2C
AuxiliaryTask gravityDownTask;		// Auxiliary task to read gravity from I2C

int readCount = 0;			// How long until we read again...
int readIntervalSamples = 0; // How many samples between reads

int printThrottle = 0; // used to limit printing frequency

// function declarations
void readIMU(void*);
void getNeutralGravity(void*);
void getDownGravity(void*);
void calibrate();
void resetOrientation();

// setup() is called once before the audio rendering starts.
// Use it to perform any initialisation and allocation which is dependent
// on the period size or sample rate.
//
// userData holds an opaque pointer to a data structure that was passed
// in from the call to initAudio().
//
// Return true on success; returning false halts the program.

bool setup(BelaContext *context, void *userData)
{
	if(!bno.begin()) {
		rt_printf("Error initialising BNO055\n");
		return false;
	}
	
	rt_printf("Initialised BNO055\n");
	
	// use external crystal for better accuracy
  	bno.setExtCrystalUse(true);
  	
	// get the system status of the sensor to make sure everything is ok
	uint8_t sysStatus, selfTest, sysError;
  	bno.getSystemStatus(&sysStatus, &selfTest, &sysError);
	rt_printf("System Status: %d (0 is Idle)   Self Test: %d (15 is all good)   System Error: %d (0 is no error)\n", sysStatus, selfTest, sysError);

	
	// set sensor reading in a separate thread
	// so it doesn't interfere with the audio processing
	i2cTask = Bela_createAuxiliaryTask(&readIMU, 5, "bela-bno");
	readIntervalSamples = context->audioSampleRate / readInterval;
	
	gravityNeutralTask = Bela_createAuxiliaryTask(&getNeutralGravity, 5, "bela-neu-gravity");
	gravityDownTask = Bela_createAuxiliaryTask(&getDownGravity, 5, "bela-down-gravity");
	
	// set up button pin
	pinMode(context, 0, buttonPin, INPUT); 
	
	// calculate inverse sample rate for synth
	gInverseSampleRate = 1.0/context->audioSampleRate;
	
	return true;
}

// render() is called regularly at the highest priority by the audio engine.
// Input and output are given from the audio hardware and the other
// ADCs and DACs (if available). If only audio is available, numAnalogFrames
// will be 0.

void render(BelaContext *context, void *userData)
{
	// iteratre through all audio samples in the frame
	for(int n = 0; n < context->audioFrames; n++) {
		// this schedules the imu sensor readings
		if(++readCount >= readIntervalSamples) {
			readCount = 0;
			Bela_scheduleAuxiliaryTask(i2cTask);
		}
		
		// print IMU values, but not every sample
		printThrottle++;
		if(printThrottle >= 500){
			rt_printf("%f %f %f\n", ypr[0], ypr[1], ypr[2]);
			imu::Vector<3> qForward = gIdleConj.toEuler();
			printThrottle = 0;
		}

		//read the value of the button
		int buttonValue = digitalRead(context, 0, buttonPin); 

		// if button wasn't pressed before and is pressed now
		if( buttonValue != lastButtonValue && buttonValue == 1 ){
			// then run calibration to set looking forward (gGravIdle) 
			// and looking down (gGravCal)
			switch(calibrationState) {
			case 0: // first time button was pressed
				setForward = 1;
				// run task to get gravity values when sensor in neutral position
				Bela_scheduleAuxiliaryTask(gravityNeutralTask);
				calibrationState = 1;	// progress calibration state
				break;
			case 1: // second time button was pressed
				// run task to get gravity values when sensor 'looking down' (for head-tracking) 
			 	Bela_scheduleAuxiliaryTask(gravityDownTask);
				calibrationState = 0; // reset calibration state for next time
				break;
			} 
		}
		lastButtonValue = buttonValue;
		
		// map yaw, pitch, or roll to frequency for synth
		// change ypr[0] to ypr[1] or ypr[2] to access the other axes
		gFrequency = map(ypr[0], M_PI*-0.5, M_PI*0.5, 100, 800);
		
		// calculate audio sample for synth
		float out = 0.3 * sinf(gPhase);
		gPhase += 2.0 * M_PI * gFrequency * gInverseSampleRate;
		if(gPhase > 2.0 * M_PI)
			gPhase -= 2.0 * M_PI;

		// write sample to output
		for(unsigned int channel = 0; channel < context->audioOutChannels; channel++) {
			audioWrite(context, n, channel, out);
		}
	}
}




// Auxiliary task to read from the I2C board
void readIMU(void*)
{
	// get calibration status
	uint8_t sys, gyro, accel, mag;
	bno.getCalibration(&sys, &gyro, &accel, &mag);
	// status of 3 means fully calibrated
	//rt_printf("CALIBRATION STATUSES\n");
	//rt_printf("System: %d   Gyro: %d Accel: %d  Mag: %d\n", sys, gyro, accel, mag);
	
	// quaternion data routine from MrHeadTracker
  	imu::Quaternion qRaw = bno.getQuat(); //get sensor raw quaternion data
  	
  	if( setForward ) {
  		gIdleConj = qRaw.conjugate(); // sets what is looking forward
  		setForward = 0; // reset flag so only happens once
  	}
		
  	steering = gIdleConj * qRaw; // calculate relative rotation data
  	quat = gCalLeft * steering; // transform it to calibrated coordinate system
  	quat = quat * gCalRight;

  	ypr = quat.toEuler(); // transform from quaternion to Euler
}

// Auxiliary task to read from the I2C board
void getNeutralGravity(void*) {
	// read in gravity value
  	imu::Vector<3> gravity = bno.getVector(I2C_BNO055::VECTOR_GRAVITY);
  	gravity = gravity.scale(-1);
  	gravity.normalize();
  	gGravIdle = gravity;
}

// Auxiliary task to read from the I2C board
void getDownGravity(void*) {
	// read in gravity value
  	imu::Vector<3> gravity = bno.getVector(I2C_BNO055::VECTOR_GRAVITY);
  	gravity = gravity.scale(-1);
  	gravity.normalize();
  	gGravCal = gravity;
  	// run calibration routine as we should have both gravity values
  	calibrate(); 
}

// calibration of coordinate system from MrHeadTracker
// see http://www.aes.org/e-lib/browse.cfm?elib=18567 for full paper
// describing algorithm
void calibrate() {
  	imu::Vector<3> g, gravCalTemp, x, y, z;
  	g = gGravIdle; // looking forward in neutral position
  
  	z = g.scale(-1); 
  	z.normalize();

  	gravCalTemp = gGravCal; // looking down
  	y = gravCalTemp.cross(g);
  	y.normalize();

  	x = y.cross(z);
  	x.normalize();

  	imu::Matrix<3> rot;
  	rot.cell(0, 0) = x.x();
  	rot.cell(1, 0) = x.y();
  	rot.cell(2, 0) = x.z();
  	rot.cell(0, 1) = y.x();
  	rot.cell(1, 1) = y.y();
  	rot.cell(2, 1) = y.z();
  	rot.cell(0, 2) = z.x();
  	rot.cell(1, 2) = z.y();
  	rot.cell(2, 2) = z.z();

  	gCal.fromMatrix(rot);

  	resetOrientation();
}

// from MrHeadTracker
// resets values used for looking forward
void resetOrientation() {
  	gCalLeft = gCal.conjugate();
  	gCalRight = gCal;
}


// cleanup() is called once at the end, after the audio has stopped.
// Release any resources that were allocated in setup().

void cleanup(BelaContext *context, void *userData)
{
	// Nothing to do here
}


/**
\imu-sine-sythn/render.cpp

Inertial Measurement Unit (IMU) Sensing with the BNO055
----------------------------------------------------------

This sketch allows you to hook up BNO055 IMU movement sensing device
to Bela, for example the Adafruit BNO055 breakout board.

To get this working with Bela you need to connect the breakout board to the I2C
terminal on the Bela board. See the Pin guide for details of which pin is which.

Connect a push button with a pull-down resistor to pin P8_08.

When running sketch, hold IMU in a neutral position and press the push button 
once. Tilt IMU down (if wearing as for head-tracking, look down) and press the 
push button a second time. The system is now calibrated. Calibration can be
run again at any time.

*/
