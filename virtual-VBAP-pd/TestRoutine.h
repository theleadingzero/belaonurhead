/*
 *  Created on: 8 September, 2018
 *      Author: Rishi Shukla
 */

#ifndef TESTROUTINE_H_
#define TESTROUTINE_H_

// BECKY - ADD AZIMUTHS HERE: range -180 (anti-clockwise) to 180 (clockwise)
int gTestAzimuth=0;

// BECKY - ADD ELEVATIONS HERE: -90 (down) to 90 (up)
int gTestElevation =-45;

//Test varaiables
float gDataOutputL[187][1536];
float gDataOutputR[187][1536];
int gColumnCount=0;
int gRowCount=0;
int gAudioCount=0;
bool gAudioOut=0;


void resetIncrement(){
  gTestElevation= -45;
  if(gTestAzimuth>-180 && gTestAzimuth <= 165) gTestAzimuth-=15;
  else if(gTestAzimuth==-180) gTestAzimuth=165;
}

void incrementLocation(){
  gTestElevation+=15;
  if(gTestAzimuth%360==0){
    if(gTestElevation>90){
      resetIncrement();
    }
  }
  else if(gTestAzimuth%60==0){
    if(gTestElevation>75){
      resetIncrement();
    }
  }
  else if(gTestAzimuth%30==0){
    if(gTestElevation>60){
      resetIncrement();
    }
  }
  else if(gTestAzimuth%15==0){
    if(gTestElevation>45){
      resetIncrement();
    }
  }
}

void writeOutput(float outL, float outR){
  if(/*gAudioOut==1 &&*/ gRowCount<187){
    gDataOutputL[gRowCount][gColumnCount] = outL;
    gDataOutputR[gRowCount][gColumnCount] = outR;
    if(++gColumnCount >=1536){
      incrementLocation();
      gColumnCount=0;
      gRowCount++;
    }
  }
  //if(++gAudioCount>=512)gAudioOut=1;
}

#endif /* TESTROUTINE_H_ */
