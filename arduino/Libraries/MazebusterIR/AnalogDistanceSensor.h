
#ifndef AnalogDistanceSensor_h
#define AnalogDistanceSensor_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "Arduino.h"
#else
	#include "WProgram.h"
	#include <pins_arduino.h>
#endif

#define __cxa_pure_virtual()

#include "FastRunningMedian.h"

/// <summary>
/// Abstract class AnalogDistanceSensor
/// </summary>
class AnalogDistanceSensor {
	public:
		void begin(int distancePin);
		
		int   getDistanceRaw();
		float getDistanceVolt();
		
		void setARefVoltage(int _refV);
		virtual float getDistanceCentimeter() = 0; 	// Is implemented in derived classes
		
	private:
		int _mapGP2Y0A21YK_V(int value);
		int _distancePin;
		
	protected:
		int _refVoltage;
};

#endif