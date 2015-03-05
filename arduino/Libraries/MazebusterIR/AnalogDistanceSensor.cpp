
#include <AnalogDistanceSensor.h>

/// <summary>
/// - int _distancePin: Number indicating the distance to an object: ANALOG IN
/// begin(): Set the input pin of the sensor
/// </summary>
void AnalogDistanceSensor::begin(int distancePin) {
	pinMode(distancePin, INPUT);
	_distancePin = distancePin;	     
	setARefVoltage(5);
}

/// <summary>
/// getDistanceRaw(): Returns the distance as a raw value: ADC output: 0 -> 1023
/// </summary>
int AnalogDistanceSensor::getDistanceRaw() {
	return (analogRead(_distancePin));
}

/// <summary>
/// getDistanceVolt(): Returns the distance as a Voltage: ADC Input: 0V -> 5V (or 0V -> 3.3V)
/// </summary>
float AnalogDistanceSensor::getDistanceVolt() {
	return (_mapGP2Y0A21YK_V(getDistanceRaw())/1000.0);
}

/// <summary>
/// _mapGP2Y0A21YKV(): Maps the ADC output to the input voltage of the ADC
/// </summary>
int AnalogDistanceSensor::_mapGP2Y0A21YK_V(int value) {
	if (_refVoltage == 3) {
		return map(value, 0, 1023, 0, 3300);
	}
	if (_refVoltage == 5) {
		return map(value, 0, 1023, 0, 5000);
	}
}

/// <summary>
/// setARefVoltage(): Set the ADC reference voltage: (Default value: 5V, set to 3 for external reference value, typically 3.3 on Arduino boards)
/// </summary>
void AnalogDistanceSensor::setARefVoltage(int refV) {
	_refVoltage = refV;
	
	if (_refVoltage == 5) {
		analogReference(DEFAULT);
	}
	if (_refVoltage == 3) {
		analogReference(EXTERNAL);
	}
}