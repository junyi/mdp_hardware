
#ifndef DistanceGP2Y0A02YK_h
#define DistanceGP2Y0A02YK_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "Arduino.h"
#else
	#include "WProgram.h"
	#include <pins_arduino.h>
#endif

#include <AnalogDistanceSensor.h>

class DistanceGP2Y0A02YK : public AnalogDistanceSensor {
	public:
		DistanceGP2Y0A02YK();
		int getDistanceCentimeter();
		int getDistanceCentimeter1();
		int getDistanceCentimeter2();
};

#endif