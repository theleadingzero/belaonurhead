/*
 *  Created on: 21 April, 2018
 *      Author: Rishi Shukla
 *****  Code extended and adapted from RTDSP module overlap add solution  *****
 */

// include files
#include <Bela.h>
#include <cmath>
#include <ne10/NE10.h>			// neon library
#include <SampleStream.h>   // adapted Bela code for streaming/processing audio
#include <SampleLoader.h>   // adapted Bela code for loading short audio files
#include <ImpulseData.h>    // distinct struct file to identify impulse responses
#include <VBAPData.h>       // lookup tables for VBAP speaker weightings
#include <StreamGainData.h> // table for audio track level balancing / muting
#include <TestRoutine.h>

// additional includes for Pd
#include <DigitalChannelManager.h>
#include <stdio.h>
#include <libpd/z_libpd.h>
extern "C" {
#include <libpd/s_stuff.h>
};
#include <UdpServer.h>
#include <Midi.h>
#include <Scope.h>
#include <string>
#include <sstream>




/*----------*/
/*----------*/
/* IMU #includes*/
#include "Bela_BNO055.h"
/*----------*/
/*----------*/

#define BUFFER_SIZE 2048   // BUFFER SIZE
#define NUM_CHANNELS 1      // NUMBER OF CHANNELS IN AUDIO STREAMS
#define NUM_STREAMS 1     // MAXIMUM NUMBER OF AUDIO STREAMS
#define NUM_SPEAKERS 8      // MAXIMUM NUMBER OF VIRTUAL SPEAKERS

extern int gSpeakers;       // Number of Speakers chosen by user
extern int gTracks;         // Concurrent tracks chosen by user
extern bool gVoiceMeta;     // Metadata playback on/off chosen by user

// instantiate the sampleStream class
SampleStream *sampleStream[NUM_STREAMS];

// instantiate binaural impulse response data buffers (left and right channels)
ImpulseData gImpulseData[NUM_SPEAKERS*2];

// global variables for stream playback code
int gStopThreads = 0;
int gTaskStopped = 0;
int gCount = 0;

// FFT overlap/add buffers and variables
float gInputBuffer[NUM_STREAMS][BUFFER_SIZE];
int gInputBufferPointer = 0;
float gOutputBufferL[BUFFER_SIZE];
float gOutputBufferR[BUFFER_SIZE];
int gOutputBufferReadPointer;
int gOutputBufferWritePointer;
int gFFTInputBufferPointer;
int gFFTOutputBufferPointer;
float *gWindowBuffer;
int gSampleCount = 0;
int gHRIRLength = 512;
int gFFTSize = 1024;
int gConvolutionSize = gFFTSize+gHRIRLength;
int gHopSize = gFFTSize/2;
float gFFTScaleFactor = 0;


// BECKY - ADD AZIMUTHS HERE: range -180 (anti-clockwise) to 180 (clockwise)
int gVBAPDefaultAzimuth[10]={-144,-72,-0,72,144,-144,-72,-0,72,144};

// BECKY - ADD ELEVATIONS HERE: -90 (down) to 90 (up)
int gVBAPDefaultElevation[10]={-10,-10,-10,-10,-10,30,30,30,30};


//Rotation variables
float gVBAPDefaultVector[NUM_STREAMS][3];
float gVBAPRotatedVector[NUM_STREAMS][3];
int gVBAPUpdatePositions[NUM_STREAMS]={0};
int gVBAPUpdateAzimuth[NUM_STREAMS]={0};
int gVBAPUpdateElevation[NUM_STREAMS]={0};
int gVBAPTracking[3]={0};

// sinewave
float gFrequencyL = 440.0;
float gFrequencyR = 230.0;
float gPhaseL;
float gPhaseR;
float gInverseSampleRate;
float gStreamPos1 = 0;


// buffers and configuration for Neon FFT processing
ne10_fft_cpx_float32_t* impulseTimeDomainL[NUM_SPEAKERS];
ne10_fft_cpx_float32_t* impulseTimeDomainR[NUM_SPEAKERS];
ne10_fft_cpx_float32_t* impulseFrequencyDomainL[NUM_SPEAKERS];
ne10_fft_cpx_float32_t* impulseFrequencyDomainR[NUM_SPEAKERS];
ne10_fft_cpx_float32_t* signalTimeDomainIn;
ne10_fft_cpx_float32_t* signalFrequencyDomain;  // (buffer to calculate L/R)
ne10_fft_cpx_float32_t* signalFrequencyDomainL;
ne10_fft_cpx_float32_t* signalFrequencyDomainR;
ne10_fft_cpx_float32_t* signalTimeDomainOutL;
ne10_fft_cpx_float32_t* signalTimeDomainOutR;
ne10_fft_cfg_float32_t cfg;

// instantialte auxiliary task to fill buffers
AuxiliaryTask gFillBuffersTask;
// instatiate auxiliary task to calculate FFTs
AuxiliaryTask gFFTTask;


// instantiate the scope
//Scope scope;

//declare process_fft_backround method
void process_fft_background(void *);
void createVectors();


/*----------*/
/*----------*/
/*IMU #variables*/

// Change this to change how often the BNO055 IMU is read (in Hz)
int readInterval = 100;

I2C_BNO055 bno; // IMU sensor object
int buttonPin = 1; // calibration button pin
int lastButtonValue = 0; // using a pulldown resistor

// Quaternions and Vectors
imu::Quaternion gCal, gCalLeft, gCalRight, gIdleConj = {1, 0, 0, 0};
imu::Quaternion qGravIdle, qGravCal, quat, steering, qRaw;

imu::Vector<3> gRaw;
imu::Vector<3> gGravIdle, gGravCal;
imu::Vector<3> ypr; //yaw pitch and roll angles


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
/*----------*/
/*----------*/


// Pd variables
float* gInBuf;
float* gOutBuf;
#define PARSE_MIDI
static std::vector<Midi*> midi;
std::vector<std::string> gMidiPortNames;

void Bela_userSettings(BelaInitSettings *settings)
{
	settings->uniformSampleRate = 1;
	settings->interleave = 0;
	settings->analogOutputsPersist = 0;
}

void dumpMidi()
{
	if(midi.size() == 0)
	{
		printf("No MIDI device enabled\n");
		return;
	}
	printf("The following MIDI devices are enabled:\n");
	printf("%4s%20s %3s %3s %s\n",
		"Num",
		"Name",
		"In",
		"Out",
		"Pd channels"
		);
	for(unsigned int n = 0; n < midi.size(); ++n)
	{
		printf("[%2d]%20s %3s %3s (%d-%d)\n", 
			n,
			gMidiPortNames[n].c_str(),
			midi[n]->isInputEnabled() ? "x" : "_",
			midi[n]->isOutputEnabled() ? "x" : "_",
			n * 16 + 1,
			n * 16 + 16
			);
	}
}

Midi* openMidiDevice(std::string name, bool verboseSuccess = false, bool verboseError = false)
{
	Midi* newMidi;
	newMidi = new Midi();
	newMidi->readFrom(name.c_str());
	newMidi->writeTo(name.c_str());
#ifdef PARSE_MIDI
	newMidi->enableParser(true);
#else
	newMidi->enableParser(false);
#endif /* PARSE_MIDI */
	if(newMidi->isOutputEnabled())
	{
		if(verboseSuccess)
			printf("Opened MIDI device %s as output\n", name.c_str());
	}
	if(newMidi->isInputEnabled())
	{
		if(verboseSuccess)
			printf("Opened MIDI device %s as input\n", name.c_str());
	}
	if(!newMidi->isInputEnabled() && !newMidi->isOutputEnabled())
	{
		if(verboseError)
			fprintf(stderr, "Failed to open  MIDI device %s\n", name.c_str());
		return nullptr;
	} else {
		return newMidi;
	}
}

static unsigned int getPortChannel(int* channel){
	unsigned int port = 0;
	while(*channel > 16){
		*channel -= 16;
		port += 1;
	}
	if(port >= midi.size()){
		// if the port number exceeds the number of ports available, send out
		// of the first port 
		rt_fprintf(stderr, "Port out of range, using port 0 instead\n");
		port = 0;
	}
	return port;
}

void Bela_MidiOutNoteOn(int channel, int pitch, int velocity) {
	int port = getPortChannel(&channel);
	rt_printf("noteout _ port: %d, channel: %d, pitch: %d, velocity %d\n", port, channel, pitch, velocity);
	midi[port]->writeNoteOn(channel, pitch, velocity);
}

void Bela_MidiOutControlChange(int channel, int controller, int value) {
	int port = getPortChannel(&channel);
	rt_printf("ctlout _ port: %d, channel: %d, controller: %d, value: %d\n", port, channel, controller, value);
	midi[port]->writeControlChange(channel, controller, value);
}

void Bela_MidiOutProgramChange(int channel, int program) {
	int port = getPortChannel(&channel);
	rt_printf("pgmout _ port: %d, channel: %d, program: %d\n", port, channel, program);
	midi[port]->writeProgramChange(channel, program);
}

void Bela_MidiOutPitchBend(int channel, int value) {
	int port = getPortChannel(&channel);
	rt_printf("bendout _ port: %d, channel: %d, value: %d\n", port, channel, value);
	midi[port]->writePitchBend(channel, value);
}

void Bela_MidiOutAftertouch(int channel, int pressure){
	int port = getPortChannel(&channel);
	rt_printf("touchout _ port: %d, channel: %d, pressure: %d\n", port, channel, pressure);
	midi[port]->writeChannelPressure(channel, pressure);
}

void Bela_MidiOutPolyAftertouch(int channel, int pitch, int pressure){
	int port = getPortChannel(&channel);
	rt_printf("polytouchout _ port: %d, channel: %d, pitch: %d, pressure: %d\n", port, channel, pitch, pressure);
	midi[port]->writePolyphonicKeyPressure(channel, pitch, pressure);
}

void Bela_MidiOutByte(int port, int byte){
	rt_printf("port: %d, byte: %d\n", port, byte);
	if(port > (int)midi.size()){
		// if the port is out of range, redirect to the first port.
		rt_fprintf(stderr, "Port out of range, using port 0 instead\n");
		port = 0;
	}
	midi[port]->writeOutput(byte);
}

void Bela_printHook(const char *received){
	rt_printf("%s", received);
}

static DigitalChannelManager dcm;

void sendDigitalMessage(bool state, unsigned int delay, void* receiverName){
	libpd_float((char*)receiverName, (float)state);
//	rt_printf("%s: %d\n", (char*)receiverName, state);
}

#define LIBPD_DIGITAL_OFFSET 11 // digitals are preceded by 2 audio and 8 analogs (even if using a different number of analogs)

void Bela_messageHook(const char *source, const char *symbol, int argc, t_atom *argv){
	if(strcmp(source, "bela_setMidi") == 0){
		int num[3] = {0, 0, 0};
		for(int n = 0; n < argc && n < 3; ++n)
		{
			if(!libpd_is_float(&argv[n]))
			{
				fprintf(stderr, "Wrong format for Bela_setMidi, expected:[hw 1 0 0(");
				return;
			}
			num[n] = libpd_get_float(&argv[n]);
		}
		std::ostringstream deviceName;
		deviceName << symbol << ":" << num[0] << "," << num[1] << "," << num[2];
		printf("Adding Midi device: %s\n", deviceName.str().c_str());
		Midi* newMidi = openMidiDevice(deviceName.str(), false, true);
		if(newMidi)
		{
			midi.push_back(newMidi);
			gMidiPortNames.push_back(deviceName.str());
		}
		dumpMidi();
		return;
	}
	if(strcmp(source, "bela_setDigital") == 0){
		// symbol is the direction, argv[0] is the channel, argv[1] (optional)
		// is signal("sig" or "~") or message("message", default) rate
		bool isMessageRate = true; // defaults to message rate
		bool direction = 0; // initialize it just to avoid the compiler's warning
		bool disable = false;
		if(strcmp(symbol, "in") == 0){
			direction = INPUT;
		} else if(strcmp(symbol, "out") == 0){
			direction = OUTPUT;
		} else if(strcmp(symbol, "disable") == 0){
			disable = true;
		} else {
			return;
		}
		if(argc == 0){
			return;
		} else if (libpd_is_float(&argv[0]) == false){
			return;
		}
		int channel = libpd_get_float(&argv[0]) - LIBPD_DIGITAL_OFFSET;
		if(disable == true){
			dcm.unmanage(channel);
			return;
		}
		if(argc >= 2){
			t_atom* a = &argv[1];
			if(libpd_is_symbol(a)){
				char *s = libpd_get_symbol(a);
				if(strcmp(s, "~") == 0  || strncmp(s, "sig", 3) == 0){
					isMessageRate = false;
				}
			}
		}
		dcm.manage(channel, direction, isMessageRate);
		return;
	}
}


void Bela_floatHook(const char *source, float value){
	/*
	 *  MODIFICATION
 	 *  ------------
	 *  Parse float sent to receiver 'tremoloRate' and assign it to a global variable
	 *  N.B. When using libpd receiver names need to be registered (see setup() function below)
	 */
	if(strncmp(source, "sourcePosition1", 11) == 0){
		gStreamPos1 = value;
		gVBAPDefaultAzimuth[0] = gStreamPos1;
		createVectors();
	}

	/*********/

	// let's make this as optimized as possible for built-in digital Out parsing
	// the built-in digital receivers are of the form "bela_digitalOutXX" where XX is between 11 and 26
	static int prefixLength = 15; // strlen("bela_digitalOut")
	if(strncmp(source, "bela_digitalOut", prefixLength)==0){
		if(source[prefixLength] != 0){ //the two ifs are used instead of if(strlen(source) >= prefixLength+2)
			if(source[prefixLength + 1] != 0){
				// quickly convert the suffix to integer, assuming they are numbers, avoiding to call atoi
				int receiver = ((source[prefixLength] - 48) * 10);
				receiver += (source[prefixLength+1] - 48);
				unsigned int channel = receiver - 11; // go back to the actual Bela digital channel number
				if(channel < 16){ //16 is the hardcoded value for the number of digital channels
					dcm.setValue(channel, value);
				}
			}
		}
	}
}

char receiverNames[16][21]={
	{"bela_digitalIn11"},{"bela_digitalIn12"},{"bela_digitalIn13"},{"bela_digitalIn14"},{"bela_digitalIn15"},
	{"bela_digitalIn16"},{"bela_digitalIn17"},{"bela_digitalIn18"},{"bela_digitalIn19"},{"bela_digitalIn20"},
	{"bela_digitalIn21"},{"bela_digitalIn22"},{"bela_digitalIn23"},{"bela_digitalIn24"},{"bela_digitalIn25"},
	{"bela_digitalIn26"}
};

static unsigned int gAnalogChannelsInUse;
static unsigned int gLibpdBlockSize;
// 2 audio + (up to)8 analog + (up to) 16 digital + 4 scope outputs
static const unsigned int gChannelsInUse = 30;
//static const unsigned int gFirstAudioChannel = 0;
static const unsigned int gFirstAnalogInChannel = 2;
static const unsigned int gFirstAnalogOutChannel = 2;
static const unsigned int gFirstDigitalChannel = 10;
static const unsigned int gFirstScopeChannel = 26;
static char multiplexerArray[] = {"bela_multiplexer"};
static int multiplexerArraySize = 0;
static bool pdMultiplexerActive = false;

#ifdef PD_THREADED_IO
void fdLoop(void* arg){
	t_pdinstance* pd_that = (t_pdinstance*)arg;
	while(!gShouldStop){
		sys_doio(pd_that);
		usleep(3000);
	}
}
#endif /* PD_THREADED_IO */

Scope scope;
unsigned int gScopeChannelsInUse = 4;
float* gScopeOut;
void* gPatch;
bool gDigitalEnabled = 0;


// function to load HRIRs for 8 virtual speakers on startup
void loadImpulse(){
  // load impulse .wav files from the relevant directory
	for(int i=0;i<gSpeakers;i++) {
		std::string speakers=to_string(gSpeakers);
		std::string number=to_string(i+1);
		std::string file= "./" + speakers + "speakers/impulse" + number + ".wav";
		const char * id = file.c_str();
    //determine the HRIR lengths (in samples) and populate the buffers
		for(int ch=0;ch<2;ch++) {
			int impulseChannel = (i*2)+ch;
			gImpulseData[impulseChannel].sampleLen = getImpulseNumFrames(id);
			gImpulseData[impulseChannel].samples = new float[getImpulseNumFrames(id)];
			getImpulseSamples(id,gImpulseData[impulseChannel].samples,ch,0, \
				gImpulseData[impulseChannel].sampleLen);
      //check buffer lengths and start/end values during setup
      /*rt_printf("Length %d = %d\n",impulseChannel, \
        gImpulseData[impulseChannel].sampleLen);
      rt_printf("Impulse %d = %f\n",impulseChannel, \
        gImpulseData[impulseChannel].samples[0]);
      rt_printf("Impulse %d = %f\n",impulseChannel, \
        gImpulseData[impulseChannel].samples[gHRIRLength-1]);*/
		}

	}
}


// function to prepare maximum number of audio streams for playback
void loadStream(){
  // load a playback .wav file into each stream buffer
	for(int k=0;k<NUM_STREAMS;k++) {
		std::string number=to_string(k+1);
		std::string file= "track" + number + ".wav";
		const char * id = file.c_str();
		sampleStream[k] = new SampleStream(id,NUM_CHANNELS,BUFFER_SIZE);
	}
}


// function to prepare FFT buffers for input signals and allocate memory
void prepFFT(){
	gFFTScaleFactor = 1.0f / (float)gConvolutionSize * 1000;
	signalTimeDomainIn = (ne10_fft_cpx_float32_t*) NE10_MALLOC (gConvolutionSize * \
		sizeof (ne10_fft_cpx_float32_t));
	signalFrequencyDomain = (ne10_fft_cpx_float32_t*) NE10_MALLOC (gConvolutionSize * \
		sizeof (ne10_fft_cpx_float32_t));
	signalFrequencyDomainL = (ne10_fft_cpx_float32_t*) NE10_MALLOC (gConvolutionSize * \
		sizeof (ne10_fft_cpx_float32_t));
	signalFrequencyDomainR = (ne10_fft_cpx_float32_t*) NE10_MALLOC (gConvolutionSize * \
		sizeof (ne10_fft_cpx_float32_t));
	signalTimeDomainOutL = (ne10_fft_cpx_float32_t*) NE10_MALLOC (gConvolutionSize * \
		sizeof (ne10_fft_cpx_float32_t));
	signalTimeDomainOutR = (ne10_fft_cpx_float32_t*) NE10_MALLOC (gConvolutionSize * \
		sizeof (ne10_fft_cpx_float32_t));
	cfg = ne10_fft_alloc_c2c_float32_neon (gConvolutionSize);
	memset(gInputBuffer, 0, BUFFER_SIZE * sizeof(float));
	memset(signalTimeDomainIn, 0, gConvolutionSize * sizeof (ne10_fft_cpx_float32_t));
	memset(signalFrequencyDomain, 0, gConvolutionSize * sizeof (ne10_fft_cpx_float32_t));
	memset(signalFrequencyDomainL, 0, gConvolutionSize * sizeof (ne10_fft_cpx_float32_t));
	memset(signalFrequencyDomainR, 0, gConvolutionSize * sizeof (ne10_fft_cpx_float32_t));
	memset(signalTimeDomainOutL, 0, gConvolutionSize * sizeof (ne10_fft_cpx_float32_t));
	memset(signalTimeDomainOutR, 0, gConvolutionSize * sizeof (ne10_fft_cpx_float32_t));
	memset(gOutputBufferL, 0, BUFFER_SIZE * sizeof(float));
	memset(gOutputBufferR, 0, BUFFER_SIZE * sizeof(float));
}


// function to generate IR frequency domain values
void transformHRIRs(){
  // allocate memory and add sample values to each HRIR FFT buffer (L and R)
	for (int i = 0; i < gSpeakers; i++){
		int impulseL = i*2;
		int impulseR = impulseL+1;
		impulseTimeDomainL[i] = (ne10_fft_cpx_float32_t*) NE10_MALLOC (gConvolutionSize \
			* sizeof (ne10_fft_cpx_float32_t));
		impulseTimeDomainR[i] = (ne10_fft_cpx_float32_t*) NE10_MALLOC (gConvolutionSize \
			* sizeof (ne10_fft_cpx_float32_t));
		impulseFrequencyDomainL[i] = (ne10_fft_cpx_float32_t*) NE10_MALLOC (gConvolutionSize \
			* sizeof (ne10_fft_cpx_float32_t));
		impulseFrequencyDomainR[i] = (ne10_fft_cpx_float32_t*) NE10_MALLOC (gConvolutionSize \
			* sizeof (ne10_fft_cpx_float32_t));

    // assign real component values from each impulse file
		for (int n = 0; n < gHRIRLength; n++)
		{
			impulseTimeDomainL[i][n].r = (ne10_float32_t) gImpulseData[impulseL].samples[n];
			impulseTimeDomainR[i][n].r = (ne10_float32_t) gImpulseData[impulseR].samples[n];
		}
    // transform to frequency domain (L and R)
		ne10_fft_c2c_1d_float32_neon(impulseFrequencyDomainL[i], impulseTimeDomainL[i], \
			cfg, 0);
		ne10_fft_c2c_1d_float32_neon(impulseFrequencyDomainR[i], impulseTimeDomainR[i], \
			cfg, 0);
	}
}


// function to fill buffers for maximum number of streams on startup
void fillBuffers(void*) {
	for(int i=0;i<NUM_STREAMS;i++) {
		if(sampleStream[i]->bufferNeedsFilled())
			sampleStream[i]->fillBuffer();
	}
}


// function to convert input stream point source locations to 3D vectors
void createVectors(){
	for(int i=0; i<NUM_STREAMS;i++){
    // convert default azi and ele to radians
		float aziRad=gVBAPDefaultAzimuth[i]*M_PI/180;
		float eleRad=gVBAPDefaultElevation[i]*M_PI/180;
    // convert co-ordinates to 3D vector values
		gVBAPDefaultVector[i][0]=cos(eleRad)*cos(aziRad);
		gVBAPDefaultVector[i][1]=cos(eleRad)*sin(aziRad);
		gVBAPDefaultVector[i][2]=sin(eleRad);
    // check default vector values on setup
		// rt_printf("\nSource %d – X: %f\t Y: %f\t Z: %f\n", i, \
			// gVBAPDefaultVector[i][0], \
			// gVBAPDefaultVector[i][1], \
			// gVBAPDefaultVector[i][2]);
	}
}


// function to rotate input stream point source locations using head-tracker
// YPR input data
void rotateVectors(){
  //calculate yaw rotation matrix values
	float yawRot[3]={0};
	float yawSin = sin(ypr[0]);
	float yawCos = cos(ypr[0]);
  //calculate pitch rotation matrix values
	float pitchRot[3]={0};
	float pitchSin = sin(-ypr[1]);
	float pitchCos = cos(-ypr[1]);
  //calculate roll rotation matrix values
	float rollRot[3]={0};
	float rollSin = sin(ypr[2]);
	float rollCos = cos(ypr[2]);
	for(int i=0; i<NUM_STREAMS;i++){
    //apply yaw rotation to source 3D vector locations
		yawRot[0] = yawCos*gVBAPDefaultVector[i][0] + -yawSin*gVBAPDefaultVector[i][1];
		yawRot[1] = yawSin*gVBAPDefaultVector[i][0] + yawCos*gVBAPDefaultVector[i][1];
		yawRot[2] = gVBAPDefaultVector[i][2];
    //apply pitch rotation to yaw rotated locations
		pitchRot[0] = pitchCos*yawRot[0] + pitchSin*yawRot[2];
		pitchRot[1] = yawRot[1];
		pitchRot[2] = -pitchSin*yawRot[0] + pitchCos*yawRot[2];
    //apply roll rotation to yaw and pitch rotated locations
		rollRot[0] = pitchRot[0];
		rollRot[1] = rollCos*pitchRot[1] + -rollSin*pitchRot[2];
		rollRot[2] = rollSin*pitchRot[1] + rollCos*pitchRot[2];
    //convert 3DoF rotated 3D vector locations to azi and ele values
		gVBAPUpdateAzimuth[i]=(int)roundf(atan2(rollRot[1],rollRot[0])*180/M_PI);
		gVBAPUpdateElevation[i]=(int)roundf(asin(rollRot[2]/(sqrt(pow(rollRot[0],2) \
			+pow(rollRot[1],2)+pow(rollRot[2],2))))*180/M_PI);
	}
  //check revised azi and ele value of first input on each refresh
  //rt_printf("Azimuth %d - Elevation %d\n",gVBAPUpdateAzimuth[0],gVBAPUpdateElevation[0]);
}


// configure Bela environment for playback
bool setup(BelaContext *context, void *userData)
{

	gInverseSampleRate = 1.0 / context->audioSampleRate;
	gPhaseL = 0.0;
	gPhaseR = 0.0;
	/*----------*/
	/*----------*/
	/*IMU #setup routine*/
	/*if(!bno.begin()) {
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
*/
	/*----------*/
	/*----------*/
	// Check Pd's version
	int major, minor, bugfix;
	sys_getversion(&major, &minor, &bugfix);
	printf("Running Pd %d.%d-%d\n", major, minor, bugfix);
	// We requested in Bela_userSettings() to have uniform sampling rate for audio
	// and analog and non-interleaved buffers.
	// So let's check this actually happened
	if(context->analogSampleRate != context->audioSampleRate)
	{
		fprintf(stderr, "The sample rate of analog and audio must match. Try running with --uniform-sample-rate\n");
		return false;
	}
	if(context->flags & BELA_FLAG_INTERLEAVED)
	{
		fprintf(stderr, "The audio and analog channels must be interleaved.\n");
		return false;
	}

	if(context->digitalFrames > 0 && context->digitalChannels > 0)
		gDigitalEnabled = 1;

	// add here other devices you need 
	gMidiPortNames.push_back("hw:1,0,0");
	//gMidiPortNames.push_back("hw:0,0,0");
	//gMidiPortNames.push_back("hw:1,0,1");

	scope.setup(gScopeChannelsInUse, context->audioSampleRate);
	gScopeOut = new float[gScopeChannelsInUse];

	// Check first of all if the patch file exists. Will actually open it later.
	char file[] = "_main.pd";
	char folder[] = "./";
	unsigned int strSize = strlen(file) + strlen(folder) + 1;
	char* str = (char*)malloc(sizeof(char) * strSize);
	snprintf(str, strSize, "%s%s", folder, file);
	if(access(str, F_OK) == -1 ) {
		printf("Error file %s/%s not found. The %s file should be your main patch.\n", folder, file, file);
		return false;
	}
	free(str);
	if(//context->analogInChannels != context->analogOutChannels ||
		context->audioInChannels != context->audioOutChannels){
		fprintf(stderr, "This project requires the number of inputs and the number of outputs to be the same\n");
		return false;
	}
	// analog setup
	gAnalogChannelsInUse = context->analogInChannels;

	// digital setup
	if(gDigitalEnabled)
	{
		dcm.setCallback(sendDigitalMessage);
		if(context->digitalChannels > 0){
			for(unsigned int ch = 0; ch < context->digitalChannels; ++ch){
				dcm.setCallbackArgument(ch, receiverNames[ch]);
			}
		}
	}

	for(unsigned int n = 0; n < gMidiPortNames.size(); ++n)
	{
	}
	unsigned int n = 0;
	while(n < gMidiPortNames.size())
	{
		Midi* newMidi = openMidiDevice(gMidiPortNames[n], false, false);
		if(newMidi)
		{
			midi.push_back(newMidi);
			++n;
		} else {
			gMidiPortNames.erase(gMidiPortNames.begin() + n);
		}
	}
	dumpMidi();

	// check that we are not running with a blocksize smaller than gLibPdBlockSize
	gLibpdBlockSize = libpd_blocksize();
	if(context->audioFrames < gLibpdBlockSize){
		fprintf(stderr, "Error: minimum block size must be %d\n", gLibpdBlockSize);
		return false;
	}

	// set hooks before calling libpd_init
	libpd_set_printhook(Bela_printHook);
	libpd_set_floathook(Bela_floatHook);
	libpd_set_messagehook(Bela_messageHook);
	libpd_set_noteonhook(Bela_MidiOutNoteOn);
	libpd_set_controlchangehook(Bela_MidiOutControlChange);
	libpd_set_programchangehook(Bela_MidiOutProgramChange);
	libpd_set_pitchbendhook(Bela_MidiOutPitchBend);
	libpd_set_aftertouchhook(Bela_MidiOutAftertouch);
	libpd_set_polyaftertouchhook(Bela_MidiOutPolyAftertouch);
	libpd_set_midibytehook(Bela_MidiOutByte);

	//initialize libpd. This clears the search path
	libpd_init();
	//Add the current folder to the search path for externals
	libpd_add_to_search_path(".");
	libpd_add_to_search_path("../pd-externals");

	libpd_init_audio(gChannelsInUse, gChannelsInUse, context->audioSampleRate);
	gInBuf = get_sys_soundin();
	gOutBuf = get_sys_soundout();

	// start DSP:
	// [; pd dsp 1(
	libpd_start_message(1);
	libpd_add_float(1.0f);
	libpd_finish_message("pd", "dsp");

	// Bind your receivers here
	libpd_bind("bela_digitalOut11");
	libpd_bind("bela_digitalOut12");
	libpd_bind("bela_digitalOut13");
	libpd_bind("bela_digitalOut14");
	libpd_bind("bela_digitalOut15");
	libpd_bind("bela_digitalOut16");
	libpd_bind("bela_digitalOut17");
	libpd_bind("bela_digitalOut18");
	libpd_bind("bela_digitalOut19");
	libpd_bind("bela_digitalOut20");
	libpd_bind("bela_digitalOut21");
	libpd_bind("bela_digitalOut22");
	libpd_bind("bela_digitalOut23");
	libpd_bind("bela_digitalOut24");
	libpd_bind("bela_digitalOut25");
	libpd_bind("bela_digitalOut26");
	libpd_bind("bela_setDigital");
	libpd_bind("bela_setMidi");

	// our binders
	libpd_bind("sourcePosition1");

	// open patch:
	gPatch = libpd_openfile(file, folder);
	if(gPatch == NULL){
		printf("Error: file %s/%s is corrupted.\n", folder, file); 
		return false;
	}

	// If the user wants to use the multiplexer capelet,
	// the patch will have to contain an array called "bela_multiplexer"
	// and a receiver [r bela_multiplexerChannels]
	if(context->multiplexerChannels > 0 && libpd_arraysize(multiplexerArray) >= 0){
		pdMultiplexerActive = true;
		multiplexerArraySize = context->multiplexerChannels * context->analogInChannels;
		// [; bela_multiplexer ` multiplexerArraySize` resize(
		libpd_start_message(1);
		libpd_add_float(multiplexerArraySize);
		libpd_finish_message(multiplexerArray, "resize");
		// [; bela_multiplexerChannels `context->multiplexerChannels`(
		libpd_float("bela_multiplexerChannels", context->multiplexerChannels);
	}

	// Tell Pd that we will manage the io loop,
	// and we do so in an Auxiliary Task
#ifdef PD_THREADED_IO
	sys_dontmanageio(1);
	AuxiliaryTask fdTask;
	fdTask = Bela_createAuxiliaryTask(fdLoop, 50, "libpd-fdTask", (void*)pd_this);
	Bela_scheduleAuxiliaryTask(fdTask);
#endif /* PD_THREADED_IO */

	//---end of Pd

	// print user command line selections
	rt_printf("Speakers: %d\t Tracks: %d\t Voice Metadata: %d\n", \
		gSpeakers,gTracks,gVoiceMeta);
	loadImpulse();    // load HRIRs
	loadStream();     // load audio streams
	prepFFT();        // set up FFT
	transformHRIRs(); // convert HRIRs to frequency domain
	getVBAPMatrix();  // import VBAP speaker gain data
	createVectors();

	// initialise FFT auxiliary task
	if((gFFTTask = Bela_createAuxiliaryTask(&process_fft_background, 90, \
		"fft-calculation")) == 0)
		return false;

	// initialise main output buffer pointers
	gOutputBufferReadPointer = 0;
	gOutputBufferWritePointer = gHopSize;

	// allocate the window buffer based on the FFT size
	gWindowBuffer = (float *)malloc(gFFTSize * sizeof(float));
	if(gWindowBuffer == 0)
		return false;

	// calculate a Hann window for overlap/add processing
	for(int n = 0; n < gFFTSize; n++) {
		gWindowBuffer[n] = 0.5f * (1.0f - cosf(2.0 * M_PI * n / (float)(gFFTSize - 1)));
	}

	// silence voice metadata streams if switched off by user input
	if(gVoiceMeta==0){
		for(int i=NUM_STREAMS/2;i<NUM_STREAMS;i++){
			gStreamGains[gTracks-1][i]=0.0;
		}
	}

	// initialise streaming auxiliary task
	if((gFillBuffersTask = Bela_createAuxiliaryTask(&fillBuffers, 89, \
		"fill-buffer")) == 0)
		return false;


	// tell the scope how many channels and the sample rate
	//scope.setup(2, context->audioSampleRate);

	return true;
}


void process_fft()
{
  // create the binaural signal for each speaker and sum to the L/R outputs
	for(int speaker=0; speaker<gSpeakers;speaker++){
    // copy individual streams into FFT buffer
		int pointer = (gFFTInputBufferPointer - gFFTSize + BUFFER_SIZE) % BUFFER_SIZE;
		for(int n = 0; n < gConvolutionSize; n++) {
      signalTimeDomainIn[n].r = 0.0;    // clear the FFT input buffers first
      signalTimeDomainIn[n].i = 0.0;
      // Add the value for each stream, taking into account VBAP speaker and
      // track gain weightings.
      if(n<gFFTSize){
      	for(int stream=0; stream<NUM_STREAMS;stream++){
      		signalTimeDomainIn[n].r += (ne10_float32_t) \
      		gInputBuffer[stream][pointer] \
      		* gVBAPGains[gVBAPUpdatePositions[stream]][speaker] * gWindowBuffer[n];
      	}
        // Update "pointer" each time and wrap it around to keep it within the
        // circular buffer.
      	pointer++;
      	if (pointer >= BUFFER_SIZE)
      		pointer = 0;
      }
  }
    // convert speaker feed to frequency domian
  ne10_fft_c2c_1d_float32_neon (signalFrequencyDomain, signalTimeDomainIn, \
  	cfg, 0);
    // convolve speaker feed to binaural signal with relevant HRIR
  for(int n=0;n<gConvolutionSize;n++){
      // left real
  	signalFrequencyDomainL[n].r = (signalFrequencyDomain[n].r * \
  		impulseFrequencyDomainL[speaker][n].r) \
  	- (signalFrequencyDomain[n].i * impulseFrequencyDomainL[speaker][n].i);
      // left imaginary
  	signalFrequencyDomainL[n].i = (signalFrequencyDomain[n].i * \
  		impulseFrequencyDomainL[speaker][n].r) \
  	+ (signalFrequencyDomain[n].r * impulseFrequencyDomainL[speaker][n].i);
      // right real
  	signalFrequencyDomainR[n].r = (signalFrequencyDomain[n].r * \
  		impulseFrequencyDomainR[speaker][n].r) \
  	- (signalFrequencyDomain[n].i * impulseFrequencyDomainR[speaker][n].i);
      // right imaginary
  	signalFrequencyDomainR[n].i = (signalFrequencyDomain[n].i * \
  		impulseFrequencyDomainR[speaker][n].r) \
  	+ (signalFrequencyDomain[n].r * impulseFrequencyDomainR[speaker][n].i);
  }
    // convert results back to time domain (left and right)
  ne10_fft_c2c_1d_float32_neon (signalTimeDomainOutL, signalFrequencyDomainL, \
  	cfg, 1);
  ne10_fft_c2c_1d_float32_neon (signalTimeDomainOutR, signalFrequencyDomainR, \
  	cfg, 1);


    // add results to left and output buffers
  pointer = gFFTOutputBufferPointer;
  for(int n=0; n<gConvolutionSize; n++) {
  	gOutputBufferL[pointer] += signalTimeDomainOutL[n].r * gFFTScaleFactor;
  	gOutputBufferR[pointer] += signalTimeDomainOutR[n].r * gFFTScaleFactor;
  	pointer++;
  	if(pointer >= BUFFER_SIZE)
  		pointer = 0;
  }
}
}


// Function to process the FFT in a thread at lower priority
void process_fft_background(void *) {
	process_fft();
}

/*-----------------------------------------------
* render()
*
-----------------------------------------------*/
void render(BelaContext *context, void *userData){
	//--Pd start
	int num;
#ifdef PARSE_MIDI
	for(unsigned int port = 0; port < midi.size(); ++port){
		while((num = midi[port]->getParser()->numAvailableMessages()) > 0){
			static MidiChannelMessage message;
			message = midi[port]->getParser()->getNextChannelMessage();
			rt_printf("On port %d (%s): ", port, gMidiPortNames[port].c_str());
			message.prettyPrint(); // use this to print beautified message (channel, data bytes)
			switch(message.getType()){
				case kmmNoteOn:
				{
					int noteNumber = message.getDataByte(0);
					int velocity = message.getDataByte(1);
					int channel = message.getChannel();
					libpd_noteon(channel + port * 16, noteNumber, velocity);
					break;
				}
				case kmmNoteOff:
				{
					/* PureData does not seem to handle noteoff messages as per the MIDI specs,
					 * so that the noteoff velocity is ignored. Here we convert them to noteon
					 * with a velocity of 0.
					 */
					int noteNumber = message.getDataByte(0);
					// int velocity = message.getDataByte(1); // would be ignored by Pd
					int channel = message.getChannel();
					libpd_noteon(channel + port * 16, noteNumber, 0);
					break;
				}
				case kmmControlChange:
				{
					int channel = message.getChannel();
					int controller = message.getDataByte(0);
					int value = message.getDataByte(1);
					libpd_controlchange(channel + port * 16, controller, value);
					break;
				}
				case kmmProgramChange:
				{
					int channel = message.getChannel();
					int program = message.getDataByte(0);
					libpd_programchange(channel + port * 16, program);
					break;
				}
				case kmmPolyphonicKeyPressure:
				{
					int channel = message.getChannel();
					int pitch = message.getDataByte(0);
					int value = message.getDataByte(1);
					libpd_polyaftertouch(channel + port * 16, pitch, value);
					break;
				}
				case kmmChannelPressure:
				{
					int channel = message.getChannel();
					int value = message.getDataByte(0);
					libpd_aftertouch(channel + port * 16, value);
					break;
				}
				case kmmPitchBend:
				{
					int channel = message.getChannel();
					int value =  ((message.getDataByte(1) << 7)| message.getDataByte(0)) - 8192;
					libpd_pitchbend(channel + port * 16, value);
					break;
				}
				case kmmSystem:
				// currently Bela only handles sysrealtime, and it does so pretending it is a channel message with no data bytes, so we have to re-assemble the status byte
				{
					int channel = message.getChannel();
					int status = message.getStatusByte();
					int byte = channel | status;
					libpd_sysrealtime(port, byte);
					break;
				}
				case kmmNone:
				case kmmAny:
				break;
			}
		}
	}
#else
	int input;
	for(unsigned int port = 0; port < NUM_MIDI_PORTS; ++port){
		while((input = midi[port].getInput()) >= 0){
			libpd_midibyte(port, input);
		}
	}
#endif /* PARSE_MIDI */
	unsigned int numberOfPdBlocksToProcess = context->audioFrames / gLibpdBlockSize;

	// Remember: we have non-interleaved buffers and the same sampling rate for
	// analogs, audio and digitals
	for(unsigned int tick = 0; tick < numberOfPdBlocksToProcess; ++tick)
	{
		//audio input
		for(int n = 0; n < context->audioInChannels; ++n)
		{
			memcpy(
				gInBuf + n * gLibpdBlockSize,
				context->audioIn + tick * gLibpdBlockSize + n * context->audioFrames, 
				sizeof(context->audioIn[0]) * gLibpdBlockSize
				);
		}

		// analog input
		for(int n = 0; n < context->analogInChannels; ++n)
		{
			memcpy(
				gInBuf + gLibpdBlockSize * gFirstAnalogInChannel + n * gLibpdBlockSize,
				context->analogIn + tick * gLibpdBlockSize + n * context->analogFrames, 
				sizeof(context->analogIn[0]) * gLibpdBlockSize
				);
		}
		// multiplexed analog input
		if(pdMultiplexerActive)
		{
			// we do not disable regular analog inputs if muxer is active, because user may have bridged them on the board and
			// they may be using half of them at a high sampling-rate
			static int lastMuxerUpdate = 0;
			if(++lastMuxerUpdate == multiplexerArraySize){
				lastMuxerUpdate = 0;
				libpd_write_array(multiplexerArray, 0, (float *const)context->multiplexerAnalogIn, multiplexerArraySize);
			}
		}

		unsigned int digitalFrameBase = gLibpdBlockSize * tick;
		unsigned int j;
		unsigned int k;
		float* p0;
		float* p1;
		// digital input
		if(gDigitalEnabled)
		{
			// digital in at message-rate
			dcm.processInput(&context->digital[digitalFrameBase], gLibpdBlockSize);

			// digital in at signal-rate
			for (j = 0, p0 = gInBuf; j < gLibpdBlockSize; j++, p0++) {
				unsigned int digitalFrame = digitalFrameBase + j;
				for (k = 0, p1 = p0 + gLibpdBlockSize * gFirstDigitalChannel;
					k < 16; ++k, p1 += gLibpdBlockSize) {
					if(dcm.isSignalRate(k) && dcm.isInput(k)){ // only process input channels that are handled at signal rate
						*p1 = digitalRead(context, digitalFrame, k);
					}
				}
			}
		}

		libpd_process_sys(); // process the block

		// digital outputs
		if(gDigitalEnabled)
		{
			// digital out at signal-rate
			for (j = 0, p0 = gOutBuf; j < gLibpdBlockSize; ++j, ++p0) {
				unsigned int digitalFrame = (digitalFrameBase + j);
				for (k = 0, p1 = p0  + gLibpdBlockSize * gFirstDigitalChannel;
					k < context->digitalChannels; k++, p1 += gLibpdBlockSize)
				{
					if(dcm.isSignalRate(k) && dcm.isOutput(k)){ // only process output channels that are handled at signal rate
						digitalWriteOnce(context, digitalFrame, k, *p1 > 0.5);
					}
				}
			}

			// digital out at message-rate
			dcm.processOutput(&context->digital[digitalFrameBase], gLibpdBlockSize);
		}

		// scope output
		for (j = 0, p0 = gOutBuf; j < gLibpdBlockSize; ++j, ++p0) {
			for (k = 0, p1 = p0 + gLibpdBlockSize * gFirstScopeChannel; k < gScopeChannelsInUse; k++, p1 += gLibpdBlockSize) {
				gScopeOut[k] = *p1;
			}
			scope.log(gScopeOut[0], gScopeOut[1], gScopeOut[2], gScopeOut[3]);
		}

		// audio output
		// check if buffers need filling using low priority auxiliary task
		Bela_scheduleAuxiliaryTask(gFillBuffersTask);

		// process the next audio block
		for (j = 0, p0 = gOutBuf; j < gLibpdBlockSize; j++, p0++) {
			/*----------*/
			/*----------*/
			/*IMU #setup routine*/

			// this schedules the imu sensor readings
			/*if(++readCount >= readIntervalSamples) {
		  		readCount = 0;
		  		Bela_scheduleAuxiliaryTask(i2cTask);
			}

			// print IMU values, but not every sample
			printThrottle++;
			if(printThrottle >= 4100){
				//rt_printf("Tracker Value: %d %d %d \n",gVBAPTracking[0],gVBAPTracking[1],gVBAPTracking[2]); //print horizontal head-track value
				//rt_printf("%f %f %f\n", ypr[0], ypr[1], ypr[2]);
				//rt_printf("Positions Update: %d %d\n",gVBAPUpdatePositions[0],gVBAPUpdatePositions[9]); //print horizontal head-track value
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

			lastButtonValue = buttonValue;*/
			/*----------*/
			/*----------*/



			// process and read frames for each sampleStream object into input buffer
			for(int i=0; i<NUM_STREAMS; i++){
				sampleStream[i]->processFrame();
				gInputBuffer[i][gInputBufferPointer] = sampleStream[i]->getSample(0);
			}
			//copy output buffer L/R to audio output L/R
			// for(int channel = 0; channel < context->audioOutChannels; channel++) {
			// 	if(channel == 0) {
			// 		context->audioOut[n * context->audioOutChannels + channel] = \
			// 		gOutputBufferL[gOutputBufferReadPointer];
			// 	}
			// 	else if (channel == 1){
			// 		context->audioOut[n * context->audioOutChannels + channel] = \
			// 		gOutputBufferR[gOutputBufferReadPointer];
			// 	}
			// }

			// write the audio out
			for (k = 0, p1 = p0; k < context->audioOutChannels; k++, p1 += gLibpdBlockSize) {
				// *p1 here is the sample in Pd's buffer which
				// corresponds to the jth frame of the kth channel
				// we edit its value in place
				if(k==0)
					*p1 = gOutputBufferL[gOutputBufferReadPointer];
				if(k==1)
					*p1 = gOutputBufferR[gOutputBufferReadPointer];
			}

			// log the oscillators to the scope
			//scope.log(context->audioOut[0],context->audioOut[1]);

			/*--- SCRIPT TO ENABLE TEST MODE ---
			gVBAPUpdatePositions[0]=((gTestElevation+90)*361)+gTestAzimuth+180;
			writeOutput(gOutputBufferL[gOutputBufferReadPointer], \
			gOutputBufferR[gOutputBufferReadPointer]);
			--- ---*/

			// clear the output samples in the buffers so they're ready for the next ola
			gOutputBufferL[gOutputBufferReadPointer] = 0;
			gOutputBufferR[gOutputBufferReadPointer] = 0;

			// advance the output buffer pointer
			gOutputBufferReadPointer++;
			if(gOutputBufferReadPointer >= BUFFER_SIZE)
				gOutputBufferReadPointer = 0;

			// advance the write pointer
			gOutputBufferWritePointer++;
			if(gOutputBufferWritePointer >= BUFFER_SIZE)
				gOutputBufferWritePointer = 0;

			// advance the read pointer
			gInputBufferPointer++;
			if(gInputBufferPointer >= BUFFER_SIZE)
				gInputBufferPointer = 0;

			// Increment gSampleCount and check if it reaches the hop size.
			// If so, reset buffer pointers, run the FFT and reset gSampleCount.
			gSampleCount++;
			if(gSampleCount >= gHopSize) {
				gFFTInputBufferPointer = gInputBufferPointer;
				gFFTOutputBufferPointer = gOutputBufferWritePointer;
				rotateVectors();
				// calcuate the rotated position for each stream
				for(unsigned int i=0; i < NUM_STREAMS; i++){
					gVBAPUpdatePositions[i]=((gVBAPUpdateElevation[i]+90)*361)+gVBAPUpdateAzimuth[i]+180;
				}
				Bela_scheduleAuxiliaryTask(gFFTTask);
				gSampleCount = 0;
			}
		}// end of audio output

		for(int n = 0; n < context->audioInChannels; ++n)
		{
			memcpy(
				context->audioOut + tick * gLibpdBlockSize + n * context->audioFrames, 
				gOutBuf + n * gLibpdBlockSize,
				sizeof(context->audioOut[0]) * gLibpdBlockSize
			);
		}

		

		//analog output
		for(int n = 0; n < context->analogOutChannels; ++n)
		{
			memcpy(
				context->analogOut + tick * gLibpdBlockSize + n * context->analogFrames, 
				gOutBuf + gLibpdBlockSize * gFirstAnalogOutChannel + n * gLibpdBlockSize,
				sizeof(context->analogOut[0]) * gLibpdBlockSize
			);
		}
	} // end of tick
} // end of function


/*----------*/
/*----------*/
/* Auxiliary task to read from the I2C board*/
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
/*----------*/
/*----------*/

// Clear all input buffers
  void cleanup(BelaContext *context, void *userData)
  {
  	for(int i=0;i<NUM_STREAMS;i++) {
  		delete sampleStream[i];
  	}
  	for(int i=0;i<NUM_SPEAKERS;i++) {
  		NE10_FREE(impulseTimeDomainL[i]);
  		NE10_FREE(impulseTimeDomainR[i]);
  		NE10_FREE(impulseFrequencyDomainL[i]);
  		NE10_FREE(impulseFrequencyDomainR[i]);
  	}
  	NE10_FREE(signalTimeDomainIn);
  	NE10_FREE(signalFrequencyDomain);
  	NE10_FREE(signalFrequencyDomainL);
  	NE10_FREE(signalFrequencyDomainR);
  	NE10_FREE(signalTimeDomainOutL);
  	NE10_FREE(signalTimeDomainOutR);
  	NE10_FREE(cfg);

  	std::ofstream OutL("testImpL.csv");
  	std::ofstream OutR("testImpR.csv");
  	for (int i = 0; i < 187; i++) {
  		for (int j = 0; j < gConvolutionSize; j++){
  			OutL << (float)gDataOutputL[i][j] << ',';
  			OutR << (float)gDataOutputR[i][j] << ',';
  		}
  		OutL << '\n';
  		OutR << '\n';
  	}

  	//--Pd start
  	for(auto a : midi)
	{
		delete a;
	}
	libpd_closefile(gPatch);
	delete [] gScopeOut;
	//--Pd end
  }
