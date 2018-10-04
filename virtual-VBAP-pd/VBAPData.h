/*
 *  Created on: 21 April, 2018
 *      Author: Rishi Shukla
 */
// Adapted from http://www.cplusplus.com/forum/unices/112048/
#ifndef VBAPDATA_H_
#define VBAPDATA_H_

// Two matrices of VBAP gain values to generated required source locations
// using either 4 or 8 virtual speakers.

#include <iostream>
#include <fstream>
#include <sstream>

float gVBAPGains[65341][8]={0};

void getVBAPMatrix()
{

    std::ifstream file("VBAPArray.csv");

    for(int row = 0; row < 65340; ++row)
    {
        std::string line;
        std::getline(file, line);
        if ( !file.good() )
            break;

        std::stringstream iss(line);

        for (int col = 0; col < 8; ++col)
        {
            std::string val;
            std::getline(iss, val, ',');
            if ( iss.bad() )
                break;

            std::stringstream convertor(val);
            convertor >> gVBAPGains[row][col];
        }
    }
    std::cout << "Test output: " << gVBAPGains[32670][1];
}

// Rows 673, 688, 694, 700, 715, 892, 907, 913, 919, 934
float gVBAPGains8Speakers[10][8]={
	{0.0, 0.0, 0.0, 0.0, 0.0, 0.5774, 0.8165, 0.0}, // -105°x0°
	{0.8165, 0.0, 0.0, 0.0, 0.0, 0.0, 0.5774, 0.0}, // -30°x0°
	{1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, // 0°x0°
	{0.8165, 0.0, 0.5774, 0.0, 0.0, 0.0, 0.0, 0.0}, // 30°x0°
	{0.0, 0.0, 0.8165, 0.5774, 0.0, 0.0, 0.0, 0.0}, // 105°x0°
	{0.0, 0.0, 0.0, 0.0, 0.0, 0.6306, 0.1260, 0.7658}, // -105° azimuth, 30° elevation
	{0.5774, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.8165}, // -30° azimuth, 30° elevation
	{0.7764, 0.4456, 0.0, 0.0, 0.0, 0.0, 0.0, 0.4456}, // 0° azimuth, 30° elevation
	{0.5774, 0.8165, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, // 30° azimuth, 30° elevation
	{0.0, 0.7658, 0.1260, 0.6306, 0.0, 0.0, 0.0, 0.0} // 105° azimuth, 30° elevation
};
float gVBAPGains4Speakers[10][4]={
	{0.0, 0.0, 0.6676, 0.7446}, // -105°x0°
	{0.0, 0.2588, 0.0, 0.9659}, // -30°x0°
	{0.0, 0.7071, 0.0, 0.7071}, // 0°x0°
	{0.0, 0.9659, 0.0, 0.2588}, // 30°x0°
	{0.0, 0.7446, 0.6676, 0.0}, // 105°x0°
	{0.3396, 0.0, 0.7496, 0.5682}, // -105° azimuth, 30° elevation
	{0.7419, 0.0, 0.1920, 0.6425}, // -30° azimuth, 30° elevation
	{0.8881, 0.3251, 0.0, 0.3251}, // 0° azimuth, 30° elevation
	{0.7419, 0.6425, 0.1920, 0.0}, // 30° azimuth, 30° elevation
	{0.3396, 0.5682, 0.7496, 0.0} // 105° azimuth, 30° elevation
};


#endif /* VBAPDATA_H_ */
