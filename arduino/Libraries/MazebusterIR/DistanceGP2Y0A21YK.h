
#ifndef DistanceGP2Y0A21YK_h
#define DistanceGP2Y0A21YK_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "Arduino.h"
#else
	#include "WProgram.h"
	#include <pins_arduino.h>
#endif

#include <AnalogDistanceSensor.h>
#include <ModeFilter.h>

// #include "FastRunningMedian.h"

class DistanceGP2Y0A21YK : public AnalogDistanceSensor {
	public:
		DistanceGP2Y0A21YK(int label);
		float getDistance();
		float getDistance2();
		float getDistanceMedian();
		float getDistanceMedian2();
		float getDistanceMedianStable(bool clearSmooth = true);
		void  resetSmoothing();
		float getDistanceCm();
		static float mapf(float v, float s1, float e1, float s2, float e2){
			if(fabs(e1-s1) < 1E-7)
				return e1;
			else
				return (v-s1)/(e1-s1)*(e2-s2) + s2;
		}
	private:
		int _label;
    	float filterVal = 0.5;       // this determines smoothness  - .0001 is max  1 is off (no smoothing)
    	float filterVal2 = 0.9;       // this determines smoothness  - .0001 is max  1 is off (no smoothing)
	    float smoothedVal = 0;     // this holds the last loop value just use a unique variable for every different sensor that needs smoothing
	    float smoothedVal2 = 0;     // this holds the last loop value just use a unique variable for every different sensor that needs smoothing
	    float smoothedValStable = 0;

		float getDistanceCentimeter();
		float getDistanceCentimeter2();
		float smooth(float data, float filterVal, float smoothedVal);
};

#endif
