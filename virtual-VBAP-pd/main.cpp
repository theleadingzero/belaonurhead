/*
 *  Created on: 21 April, 2018
 *      Author: Rishi Shukla
 *****  Code extended and adapted from RTDSP module Assignment 1  *****
 */

// include files
#include <unistd.h>
#include <iostream>
#include <cstdlib>
#include <libgen.h>
#include <signal.h>
#include <getopt.h>
#include <Bela.h>

using namespace std;

int gSpeakers=8;	// global variable to store VBAP speaker setup (4 or 8)
int gTracks=5;		// global variable to store number of tracks played back (1 to 5)
bool gVoiceMeta=1;// global variable to store voice metadate state (off or on)

// Handle Ctrl-C by requesting that the audio rendering stop
void interrupt_handler(int var)
{
	gShouldStop = true;
}

// Print usage information
void usage(const char * processName)
{
	cerr << "Usage: " << processName << " [options]" << endl;
	Bela_usage();
	cerr << "   --help [-h]:                Print this menu\n";
}

int main(int argc, char *argv[])
{
	BelaInitSettings settings;		// standard audio settings
	int format;
	struct option customOptions[] =
	{
		{"help", 0, NULL, 'h'},
		{"speakers", 1, NULL, 's'},	// argument for speaker number
		{"tracks", 1, NULL, 't'},		// argument for track count
		{"voice", 1, NULL, 'm'},		// argument for voice metadata
		{NULL, 0, NULL, 0}
	};

	// Set default settings
	Bela_defaultSettings(&settings);
	settings.setup = setup;
	settings.render = render;
	settings.cleanup = cleanup;

	// Parse command-line arguments
	while (1) {
		int c;
		if ((c = Bela_getopt_long(argc, argv, "hf:", customOptions, &settings)) < 0)
				break;
		switch (c) {
		case 'h':
				usage(basename(argv[0]));
				exit(0);
    case 's':
				// read speaker argument and force to required values
    		gSpeakers=atoi(optarg);
				if(gSpeakers<4)
				gSpeakers=4;
				if(gSpeakers>4)
				gSpeakers=8;
    		break;
		case 't':
				// read track playback argument and force to required values
		    gTracks=atoi(optarg);
				if(gTracks<1)
				gTracks=1;
				if(gTracks>5)
				gTracks=5;
		    break;
		case 'm':
				// read voice metadata argument
				gVoiceMeta=atoi(optarg);
				break;
		case '?':
		default:
				usage(basename(argv[0]));
				exit(1);
		}
	}


	// Initialise the PRU audio device
	if(Bela_initAudio(&settings, &format) != 0) {
		cout << "Error: unable to initialise audio" << endl;
		return -1;
	}


	// Start the audio device running
	if(Bela_startAudio()) {
		cout << "Error: unable to start real-time audio" << endl;
		return -1;
	}


	// Set up interrupt handler to catch Control-C
	signal(SIGINT, interrupt_handler);
	signal(SIGTERM, interrupt_handler);


	// Run until told to stop
	while(!gShouldStop) {
		usleep(100000);
	}


	// Stop the audio device
	Bela_stopAudio();


	// Clean up any resources allocated for audio
	Bela_cleanupAudio();


	// All done!
	return 0;
}
