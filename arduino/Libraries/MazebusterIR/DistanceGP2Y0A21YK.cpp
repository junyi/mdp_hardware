
/// <summary>
/// DistanceGP2Y0A21YK.cpp - Library for retrieving data from the GP2Y0A21YK IR Distance sensor.
/// </summary>
#include <DistanceGP2Y0A21YK.h>
#define ARRAY_SIZE 8

const int adcLowerBoundary[]  = {90, 87, 135, 87, 89, 169};
const int adcUpperBoundary[]  = {492, 475, 508, 482, 432, 538};

const byte cmLowerBoundary  =  10;
const byte cmUpperBoundary  =  60;
const byte cmLowerBoundaryL  =  16;
const byte cmUpperBoundaryL  =  70;

// static int adcTable[][ARRAY_SIZE] =	{
// {757, 469, 256, 180, 137, 84, 61, 57},
// {755, 491, 261, 180, 145, 139, 118, 127},
// {767, 493, 270, 192, 161, 158, 157, 155},
// {760, 492, 268, 183, 140, 114, 101, 97},
// {765, 492, 265, 190, 152, 125, 95, 77}
// };

// static int adcTable[][ARRAY_SIZE] =	{
// {492, 389, 318, 263, 228, 201, 176, 162, 145, 133, 125, 118, 110, 106, 99, 90},
// {475, 403, 316, 254, 229, 204, 175, 159, 146, 134, 125, 120, 111, 99, 94, 87},
// {508, 420, 333, 269, 242, 218, 192, 178, 168, 154, 150, 141, 138, 137, 136, 135},
// {482, 387, 329, 280, 251, 228, 203, 183, 164, 146, 130, 117, 110, 105, 98, 87},
// {432, 365, 296, 252, 229, 208, 178, 165, 149, 131, 125, 115, 101, 96, 91, 89}
// };

// static int adcTable[][ARRAY_SIZE] =	{
// 	{631, 627, 391, 284, 224, 184, 156, 128},
// 	{631, 577, 381, 285, 231, 192, 168, 148},
// 	{643, 601, 384, 290, 239, 211, 191, 178},
// 	{641, 641, 445, 315, 244, 196, 164, 132},
// 	{641, 641, 439, 314, 240, 192, 160, 146}
// };

static int adcTable[][ARRAY_SIZE] =	{
	{640, 635, 414, 296, 237, 192, 161, 142},
	{640, 630, 423, 309, 240, 205, 176, 152},
	{640, 627, 407, 302, 246, 208, 192, 192},
	{650, 644, 447, 320, 246, 197, 162, 139},
	{650, 647, 453, 324, 252, 206, 175, 157},
};

static FastRunningMedian<0> medianCm;
static FastRunningMedian<0> median;
static FastRunningMedian<0> medianStable;
// {401, 234, 170, 131, 107,  85, 71, 57};

// static byte distanceTable[ARRAY_SIZE] =	{10, 13, 16, 20, 23, 26, 30, 33, 36, 40, 43, 46, 50, 53, 56, 60};
static float distanceTable[][ARRAY_SIZE] =	{
	{-1, 0, 5, 10, 15, 20, 25, 30},
	{-1, 0, 5, 10, 15, 20, 25, 30},
	{-1, 0, 5, 10, 15, 20, 25, 30},
	{-1, 0, 5, 10, 15, 20, 25, 30},
	{-1, 0, 5, 10, 15, 20, 25, 30}
};

/// <summary>
/// Constructor
/// </summary>
DistanceGP2Y0A21YK::DistanceGP2Y0A21YK(int label) {
	_label = label;
}

ModeFilter modeFilter;

float DistanceGP2Y0A21YK::getDistanceCm() {
	// for(int j = 0; j < 5; j++){
	// 	modeFilter = ModeFilter();

	// 	for(int i = 0; i < MOD_FILTER_SIZE; i++){
	// 		int raw = getDistanceRaw();
	// 		modeFilter.insert(raw);
	// 	}

	// 	medianCm.addValue(modeFilter.mode());
	// }

	// int adcValue = medianCm.getMedian();
	modeFilter = ModeFilter();

	for(int i = 0; i < MOD_FILTER_SIZE; i++){
		int raw = getDistanceRaw();
		modeFilter.insert(raw);
	}

	int adcValue = modeFilter.mode();
	// Serial.print("Sensor: ");
	// Serial.print(_label);
	// Serial.print(" ");
	// Serial.println(adcValue);

	// if (adcValue < adcLowerBoundary[_label]) {	//Distance (in cm) is inversely related to ADC values
	// 	return (cmUpperBoundary);
	// }
	
	// if (adcValue >= adcUpperBoundary[_label] ) {
	// 	return (cmLowerBoundary);
	// }

	// for (byte x = 1; x < ARRAY_SIZE; x++) {
	// 	if (adcValue >= adcTable[_label][x]) {
	// 		return DistanceGP2Y0A21YK::map(adcValue, adcTable[_label][x-1], adcTable[_label][x], distanceTable[x-1], distanceTable[x]);
	// 	}
	// }

	if (adcValue < adcTable[_label][ARRAY_SIZE-1]) {	//Distance (in cm) is inversely related to ADC values
		return distanceTable[_label][ARRAY_SIZE-1];
	}
	
	if (adcValue >= adcTable[_label][0] ) {
		return distanceTable[_label][0];
	}

	if(_label <= 4){
		for (byte x = 1; x < ARRAY_SIZE; x++) {
			if (adcValue >= adcTable[_label][x]) {
				float dist = DistanceGP2Y0A21YK::mapf(adcValue, adcTable[_label][x-1], adcTable[_label][x], distanceTable[_label][x-1], distanceTable[_label][x]);
				return dist;
			}
		}
		// return pow(adcValue/4416.52599835545, 1/-0.947547160316633);
	}
	return -1;
	// else if(_label == 1){
	// 	return pow(adcValue/4120.72436828283, 1/-0.93133245362108);
	// }else if(_label == 2){
	// 	return pow(adcValue/3406.71999540593, 1/-0.836077717324656);
	// }else if(_label == 3){
	// 	return pow(adcValue/4129.96789606474, 1/-0.915472818259607);
	// }else if(_label == 4){
	// 	return pow(adcValue/4697.02274737473, 1/-0.991324543568903);

	// 	// return pow(adcValue/3566.48141266335, 1/-0.895527573090872);
	// }else if(_label == 5){
	// 	// return -8.33656097/582605E-10*pow(adcValue, 4) + 1.70099164379982E-07*pow(adcValue, 3) + 7.15664379621434E-04*pow(adcValue, 2) - 5.35245811418573E-01*adcValue + 1.40279263671649E+02;
	// 	return log(adcValue/784.026025721219)/-0.0224518497914795;
	// 	// return -4.53257242576801E-09*pow(adcValue,4) + 6.39539453305011E-06*pow(adcValue,3) - 0.0030090331992857*pow(adcValue,2) + 0.400269778414329*adcValue + 57.9204848189325;
	// }
}

void DistanceGP2Y0A21YK::resetSmoothing(){
	smoothedValStable = 0;
}

float DistanceGP2Y0A21YK::getDistanceCentimeter2() {
	modeFilter = ModeFilter();

	for(int i = 0; i < MOD_FILTER_SIZE; i++){
		int raw = getDistanceRaw();
		modeFilter.insert(raw);
	}
	
	int adcValue = modeFilter.mode();

	if (smoothedValStable == 0){
		smoothedValStable = adcValue;
	}else{
		smoothedValStable =  smooth(adcValue, 0.5, smoothedValStable);
	}

	adcValue = smoothedValStable;


	// int adcValue = getDistanceRaw();
	
	// if (adcValue < adcLowerBoundary[_label]) {	//Distance (in cm) is inversely related to ADC values
	// 	return (cmUpperBoundary);
	// }
	
	// if (adcValue >= adcUpperBoundary[_label] ) {
	// 	return (cmLowerBoundary);
	// }

	// for (byte x = 1; x < ARRAY_SIZE; x++) {
	// 	if (adcValue >= adcTable[_label][x]) {
	// 		return DistanceGP2Y0A21YK::map(adcValue, adcTable[_label][x-1], adcTable[_label][x], distanceTable[x-1], distanceTable[x]);
	// 	}
	// }

	if (adcValue < 75) {	//Distance (in cm) is inversely related to ADC values
		return (70);
	}
	
	if (adcValue >= 650 ) {
		return (7);
	}

	if(_label == 0){
		return pow(adcValue/4416.52599835545, 1/-0.947547160316633);
	}else if(_label == 1){
		return pow(adcValue/4120.72436828283, 1/-0.93133245362108);
	}else if(_label == 2){
		return pow(adcValue/3406.71999540593, 1/-0.836077717324656);
	}else if(_label == 3){
		return pow(adcValue/4129.96789606474, 1/-0.915472818259607);
	}else if(_label == 4){
		return pow(adcValue/4697.02274737473, 1/-0.991324543568903);

		// return pow(adcValue/3566.48141266335, 1/-0.895527573090872);
	}else if(_label == 5){
		// return -8.33656097/582605E-10*pow(adcValue, 4) + 1.70099164379982E-07*pow(adcValue, 3) + 7.15664379621434E-04*pow(adcValue, 2) - 5.35245811418573E-01*adcValue + 1.40279263671649E+02;
		return log(adcValue/784.026025721219)/-0.0224518497914795;
		// return -4.53257242576801E-09*pow(adcValue,4) + 6.39539453305011E-06*pow(adcValue,3) - 0.0030090331992857*pow(adcValue,2) + 0.400269778414329*adcValue + 57.9204848189325;
	}
}

float DistanceGP2Y0A21YK::getDistanceCentimeter() {
	int adcValue = getDistanceRaw();
	if (adcValue < adcLowerBoundary[_label]) {	//Distance (in cm) is inversely related to ADC values
		return _label == 5 ? cmUpperBoundaryL : cmUpperBoundary;
	}
	
	if (adcValue >= adcUpperBoundary[_label] ){
		return _label == 5 ? cmLowerBoundaryL : cmLowerBoundary;
	}

	// return 1.0/(adcValue*shortCoeff[_label - 1][1] + shortCoeff[_label - 1][0]) - short_const[_label - 1];

	if(_label != 5){
		for (byte x = 1; x < ARRAY_SIZE; x++) {
			if (adcValue >= adcTable[_label][x]) {
				return DistanceGP2Y0A21YK::mapf(adcValue, adcTable[_label][x-1], adcTable[_label][x], distanceTable[_label][x-1], distanceTable[_label][x]);
			}
		}
	}
}

float DistanceGP2Y0A21YK::getDistance2() {
	return getDistanceCentimeter2();
}

float DistanceGP2Y0A21YK::getDistance() {
	return getDistanceCentimeter();
}

float DistanceGP2Y0A21YK::getDistanceMedian2() {
	for(int i = 0; i < 20; i++){
		median.addValue(getDistance2());
	}


	smoothedVal =  smooth(median.getMedian(), filterVal, smoothedVal);
  
	// return median.getMedian();
	return smoothedVal;
}

float DistanceGP2Y0A21YK::getDistanceMedianStable(bool clearSmooth) {
	for(int i = 0; i < 50; i++){
		medianStable.addValue(getDistance2());
	}

	// if (clearSmooth)
	// 	smoothedValStable = medianStable.getMedian();
	// else
	smoothedValStable =  smooth(medianStable.getMedian(), 0.8, smoothedValStable);
  
	// return median.getMedian();
	return smoothedValStable;
}

float DistanceGP2Y0A21YK::getDistanceMedian() {
	for(int i = 0; i < 5; i++){
		median.addValue(getDistance());
	}

	if(_label != 5){
    	smoothedVal =  smooth(median.getMedian(), filterVal, smoothedVal);
		return smoothedVal;
	}
   	else{
    	smoothedVal2 =  smooth(median.getMedian(), filterVal2, smoothedVal2);
		return smoothedVal2;
   	}
  		
}


float DistanceGP2Y0A21YK::smooth(float data, float filterVal, float smoothedVal) {


  if (filterVal > 1){      // check to make sure param's are within range
    filterVal = .99;
  }
  else if (filterVal <= 0){
    filterVal = 0;
  }

  smoothedVal = (data * (1 - filterVal)) + (smoothedVal  *  filterVal);

  return smoothedVal;
}