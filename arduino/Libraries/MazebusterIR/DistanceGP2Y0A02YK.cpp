
/// <summary>
/// DistanceGP2Y0A02YK.cpp - Library for retrieving data from the GP2Y0A02YK IR Distance sensor.
/// </summary>

#include <DistanceGP2Y0A02YK.h>
#define ARRAY_SIZE 14


const int	adcLowerBoundary	=  94;
const int	adcUpperBoundary	= 518; //478;

const byte	cmLowerBoundary		=  20;
const byte	cmUpperBoundary		= 150;

const int	adcTable[ARRAY_SIZE]		=	{518, 409, 315, 253, 215, 184, 163, 145, 135, 122, 114, 104, 102,  94};
const byte	distanceTable[ARRAY_SIZE]	=	{ 20,  30,  40,  50,  60,  70,  80,  90, 100, 110, 120, 130, 140, 150};


/// <summary>
/// Constructor
/// </summary>
DistanceGP2Y0A02YK::DistanceGP2Y0A02YK() {
	
}

/// <summary>
/// getDistanceCentimeter(): Returns the distance in centimetres
/// </summary>
int DistanceGP2Y0A02YK::getDistanceCentimeter() {
	int adcValue = getDistanceRaw();
	if (adcValue < adcLowerBoundary) {	//Distance (in cm) is inversely related to ADC values
		return (cmUpperBoundary);
	}
	
	if (adcValue >= adcUpperBoundary) {
		return (cmLowerBoundary);
	}
	
	for(byte x=1; x<ARRAY_SIZE; x++) {
		if (adcValue >= adcTable[x]) {
			return map(adcValue, adcTable[x-1], adcTable[x], distanceTable[x-1], distanceTable[x]);
		}
	}
}

int DistanceGP2Y0A02YK::getDistanceCentimeter1() {
	int adcValue = getDistanceRaw();
	
	if (adcValue < adcLowerBoundary) {	//Distance (in cm) is inversely related to ADC values
		return (cmUpperBoundary);
	}
	
	if (adcValue >= adcUpperBoundary ) {
		return (cmLowerBoundary);
	}
	
	return 1/(-0.002775497 + adcValue * 8.82336E-5) - 0.3; 
	//return (10460 / (adcValue - 32));
	// return (16667.0 / (adcValue + 15)) - 10;
}

int DistanceGP2Y0A02YK::getDistanceCentimeter2() {	
	const int A = 0.0082712905;
	const int B = 939.57652;
	const int C = -3.3978697;
	const int D = 17.339222;
	
	const int minVolt = 0.40;
	const int maxVolt = 2.50;
	
	float voltage = getDistanceVolt();
	
	if(voltage < minVolt) {
		return (cmUpperBoundary);
	}
	else if(voltage > maxVolt) {
		return -1;
		//return (cmLowerBoundary);
	}
	else return ((A + B*voltage) / (1 + C*voltage + D*voltage*voltage));	//Curve fit
}

int DistanceGP2Y0A02YK::getDistanceMedian() {
	for(int i = 0; i < 32; i++){
		median.addValue(getDistance());
	}

    smoothedVal =  smooth(median.getMedian(), filterVal, smoothedVal);

    if (smoothedVal < adcLowerBoundary) {	//Distance (in cm) is inversely related to ADC values
		return (cmUpperBoundary);
	}
	
	if (smoothedVal >= adcUpperBoundary ){
		return (cmLowerBoundary);
	}

	return smoothedVal;
}


int DistanceGP2Y0A02YK::smooth(int data, float filterVal, float smoothedVal) {


  if (filterVal > 1){      // check to make sure param's are within range
    filterVal = .99;
  }
  else if (filterVal <= 0){
    filterVal = 0;
  }

  smoothedVal = (data * (1 - filterVal)) + (smoothedVal  *  filterVal);

  return (int)smoothedVal;
}