/*
 *  Created on: 21 April, 2018
 *      Author: Rishi Shukla
 */

#ifndef StreamGainDATA_H_
#define StreamGainDATA_H_

// A matrix of gains values to balance and mute audio streams for demo purposes
float gStreamGains[5][10]={
	{0.0,0.0,0.71,0.0,0.0,0.0,0.0,0.39,0.0,0.0}, // one track (centre)
	{0.0,1.0,0.0,0.52,0.0,0.0,0.39,0.0,0.39,0.0}, // two tracks (front L/R)
	{0.77,0.0,0.71,0.0,0.71,0.39,0.0,0.39,0.0,0.39}, // three tracks (centre & back L/R)
	{0.77,1.0,0.0,0.52,0.71,0.39,0.39,0.0,0.39,0.39}, // four tracks (no centre)
	{0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5} // five tracks (all)
};

#endif /* StreamGainDATA_H_ */
