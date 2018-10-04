/***** Code taken from Bela SampleLoader example *****/
/*
 * SampleData.h
 *
 *  Created on: Nov 5, 2014
 *      Author: Victor Zappi
 */

#ifndef SAMPLEDATA_H
#define SAMPLEDATA_H

// User defined structure to pass between main and render complex data retrieved from file
struct SampleData {
	float *samples;	// Samples in file
	int sampleLen;	// Total num of samples
};



#endif /* SAMPLEDATA_H */
