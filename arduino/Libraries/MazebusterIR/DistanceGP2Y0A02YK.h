
#ifndef DistanceGP2Y0A02YK_h
#define DistanceGP2Y0A02YK_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "Arduino.h"
#else
	#include "WProgram.h"
	#include <pins_arduino.h>
#endif

#include <AnalogDistanceSensor.h>
#include "FastRunningMedian.h"

class DistanceGP2Y0A02YK : public AnalogDistanceSensor {
	public:
		DistanceGP2Y0A02YK(int label);
		int getDistance();
		int getDistanceMedian();
	private:
		int _label;
 		FastRunningMedian<int, 100, 0> median;
    	float filterVal = 0.8;       // this determines smoothness  - .0001 is max  1 is off (no smoothing)
	    float smoothedVal = 0;     // this holds the last loop value just use a unique variable for every different sensor that needs smoothing

		int getDistanceCentimeter();
		int getDistanceCentimeter2();
		int smooth(int data, float filterVal, float smoothedVal);
};

#endif
