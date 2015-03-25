#ifndef ModeFilter_h
#define ModeFilter_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "Arduino.h"
#else
	#include "WProgram.h"
	#include <pins_arduino.h>
#endif

#define MOD_FILTER_SIZE 30

#include <inttypes.h>
#include <HashMap.h>

class ModeFilter
{
  private:
  public:
		ModeFilter();
		void             insert(int _sample);
		int              mode();

  private:
		void 			quickSort(int arr[], int left, int right);
		HashMap<int,int,MOD_FILTER_SIZE> hashmap;
};

#endif