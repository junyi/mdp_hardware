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

ModeFilter::ModeFilter()
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
}

//Mode function, returning the mode or median.
int ModeFilter::mode(){
        int mode = hashmap.keyAt(0);
        int maxCount = hashmap.valueAt(0);

        for(int i = 1; i < MOD_FILTER_SIZE; i++){
                int value = hashmap.valueAt(i);
                // Serial.print(hashmap.keyAt(i));
                // Serial.print(" ");
                // Serial.println(hashmap.valueAt(i));
                if(value >= maxCount){
                        maxCount = value;
                        mode = hashmap.keyAt(i);
                }
        }

        hashmap.clear();


        return mode;
      
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