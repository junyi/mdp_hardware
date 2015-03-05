
/// <summary>
/// DistanceGP2Y0A21YK.cpp - Library for retrieving data from the GP2Y0A21YK IR Distance sensor.
/// </summary>
#include <DistanceGP2Y0A21YK.h>
#define ARRAY_SIZE 7

const int adcLowerBoundary  =  15;
const int adcUpperBoundary  = 485;

const byte cmLowerBoundary  =  10;
const byte cmUpperBoundary  =  70;

static int adcTable[ARRAY_SIZE] =		{485, 279, 203, 170, 120, 46, 15};
// {401, 234, 170, 131, 107,  85, 71, 57};

static byte distanceTable[ARRAY_SIZE] =	{ 10,  20,  30,  40,  50,  60, 70};

static double shortCoeff[][2] = {
    {0.002339866, 0.000150535},
    {0, 0},
    {-0.007050515, 0.000246948},
    {0.000452291, 0.000197229}
};

static double short_const[4] = {6, 0, 0.2, 2};

/// <summary>
/// Constructor
/// </summary>
DistanceGP2Y0A21YK::DistanceGP2Y0A21YK(int label) {
	_label = label;
}

/// <summary>
/// getDistanceCentimeter(): Returns the distance in centimetres
/// </summary>
//Detect range from 10cm - 80cm
int DistanceGP2Y0A21YK::getDistanceCentimeter2() {
	int adcValue = getDistanceRaw();
	
	if (adcValue < adcLowerBoundary) {	//Distance (in cm) is inversely related to ADC values
		return (cmUpperBoundary);
	}
	
	if (adcValue >= adcUpperBoundary ) {
		return (cmLowerBoundary);
	}

	for (byte x = 1; x < ARRAY_SIZE; x++) {
		if (adcValue >= adcTable[x]) {
			return map(adcValue, adcTable[x-1], adcTable[x], distanceTable[x-1], distanceTable[x]);
		}
	}
}

double DistanceGP2Y0A21YK::getDistanceCentimeter() {
	if (adcValue < adcLowerBoundary) {	//Distance (in cm) is inversely related to ADC values
		return (cmUpperBoundary);
	}
	
	if (adcValue >= adcUpperBoundary ){
		return (cmLowerBoundary);
	}

	return 1/(adcValue - shortCoeff[label - 1] - short_const[label - 1])
}

double DistanceGP2Y0A21YK::getDistance() {
	if (_label == 2) {
		return getDistanceCentimeter2();
	} else {
		return getDistanceCentimeter();
	}
}

double DistanceGP2Y0A21YK::getDistanceMedian() {
	for(int i = 0; i < 100; i++){
		median.addValue(getDistance());
	}

    smoothedVal =  smooth(median.getMedian(), filterVal, smoothedVal);

	return smoothedVal;
}

int DistanceGP2Y0A21YK::smooth(int data, float filterVal, float smoothedVal) {


  if (filterVal > 1){      // check to make sure param's are within range
    filterVal = .99;
  }
  else if (filterVal <= 0){
    filterVal = 0;
  }

  smoothedVal = (data * (1 - filterVal)) + (smoothedVal  *  filterVal);

  return (int)smoothedVal;
}