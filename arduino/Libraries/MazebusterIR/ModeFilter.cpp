/*
        ModeFilter.cpp - Mode Filter Library for Ardupilot Mega. Arduino
        Code by Jason Short. DIYDrones.com
        Adapted from code by Jason Lessels(June 6, 2011), Bill Gentles (Nov. 12, 2010)


        This library is free software; you can redistribute it and/or
        modify it under the terms of the GNU Lesser General Public
        License as published by the Free Software Foundation; either
        version 2.1 of the License, or (at your option) any later version.


*/
#include "ModeFilter.h"

// Constructors ////////////////////////////////////////////////////////////////
static HashMap<int, int, MOD_FILTER_SIZE> hashmap;

ModeFilter::ModeFilter() :
        _sample_index(0)
{
}

// Public Methods //////////////////////////////////////////////////////////////
        //Sorting function
        // sort function (Author: Bill Gentles, Nov. 12, 2010)
        //      *a is an array pointer function

void ModeFilter::insert(int _sample){
        if (!hashmap.contains(_sample)){
                hashmap[_sample] = 1;
        }else{
                hashmap[_sample]++;
        }
        // _samples[_sample_index] = _sample;

        // _sample_index++;

        // if (_sample_index >= MOD_FILTER_SIZE)
        //         _sample_index = 0;

}

//Mode function, returning the mode or median.
int16_t ModeFilter::mode(){
        int mode = hashmap.keyAt(0);
        int maxCount = hashmap.valueAt(0);

        for(int i = 1; i < MOD_FILTER_SIZE; i++){
                int value = hashmap.valueAt(i);

                if(value >= maxCount){
                        maxCount = value;
                        mode = hashmap.keyAt(i);
                }
        }

        hashmap.clear();

        return mode;
        // quickSort(_samples, 0, MOD_FILTER_SIZE - 1);

        // int fmode               = 0;
        // int i                  = 0;
        // int count              = 0;
        // int maxCount   = 0;
        // int bimodal    = 0;

        // while(count > maxCount){
        //         fmode           = _samples[i];
        //         maxCount        = count;
        //         bimodal         = 0;
        // }

        // if(count == 0) i++;

        // if(count == maxCount){ //If the dataset has 2 or more modes.
        //         bimodal = 1;
        // }

        // if(fmode == 0 || bimodal == 1){ //Return the median if there is no mode.
        //         fmode = _samples[(MOD_FILTER_SIZE / 2)];
        // }

        // return fmode;
}

 void ModeFilter::quickSort(int arr[], int left, int right) {
      int i = left, j = right;
      int tmp;
      int pivot = arr[(left + right) / 2];
 
      /* partition */
      while (i <= j) {
            while (arr[i] < pivot)
                  i++;
            while (arr[j] > pivot)
                  j--;
            if (i <= j) {
                  tmp = arr[i];
                  arr[i] = arr[j];
                  arr[j] = tmp;
                  i++;
                  j--;
            }
      };
 
      /* recursion */
      if (left < j)
            quickSort(arr, left, j);
      if (i < right)
            quickSort(arr, i, right);
}