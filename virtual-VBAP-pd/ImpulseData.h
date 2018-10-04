/*
 *  Created on: 21 April, 2018
 *      Author: Rishi Shukla
 ****** Code adapted from Bela SampleLoader example  *****
 */

#ifndef IMPULSEDATA_H_
#define IMPULSEDATA_H_

// User defined structure to pass between main and render complex data retrieved from file
struct ImpulseData {
	float *samples;	// Samples in file
	int sampleLen;	// Total num of samples
};



#endif /* IMPULSEDATA_H_ */
