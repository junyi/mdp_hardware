/************************************************************************************************************
 * SharpIR.h - Arduino library for retrieving distance (in cm) from the analog GP2Y0A21Y and GP2Y0A02YK     *
 * Distance sensors                                                                                         *
 * Copyright 2014 Dr. Marcal Casas-Cartagena (marcal.casas@gmail.com)                                       *
 * Last update: 07.01.2014                                                                                  *
 ************************************************************************************************************
 
 ************************************************************************************************************
 * This library is free software; you can redistribute it and/or                                            *
 * modify it under the terms of the GNU Lesser General Public                                               *
 * License as published by the Free Software Foundation; either                                             *
 * version 2.1 of the License, or (at your option) any later version.                                       *
 *                                                                                                          *
 * This library is distributed in the hope that it will be useful,                                          *
 * but WITHOUT ANY WARRANTY; without even the implied warranty of                                           *
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU                                        *
 * Lesser General Public License for more details.                                                          *
 *                                                                                                          *
 * You should have received a copy of the GNU Lesser General Public                                         *
 * License along with this library; if not, write to the Free Software                                      *
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA                               *
 ***********************************************************************************************************/


// The Sahrp IR sensors are cheap but somehow unreliable. I've found that when doing continous readings to a
// fix object, the distance given oscilates quite a bit from time to time. For example I had an object at
// 31 cm. The readings from the sensor were mainly steady at the correct distance but eventually the distance
// given dropped down to 25 cm or even 16 cm. That's quite a bit and for some applications it is quite
// unacceptable. I checked the library http://code.google.com/p/gp2y0a21yk-library/ by Jeroen Doggen
// (jeroendoggen@gmail.com) and what the author was doing is to take a bunch of readings and give an average of them

// The present library works similary. It reads a bunch of readings (avg), it checks if the current reading
// differs a lot from the previous one (tolerance) and if it doesn't differ a lot, it takes it into account
// for the mean distance.
// The distance is calculated from a formula extracted from the graphs on the sensors datasheets
// After some tests, I think that a set of 20 to 25 readings is more than enough to get an accurate distance
// Reading 25 times and return a mean distance takes 53 ms. For my application of the sensor is fast enough.
// This library has the formulas to work with the GP2Y0A21Y and the GP2Y0A02YK sensors but exanding it for
// other sensors is easy enough.


#include "Arduino.h"
#include "MazebusterIR.h"
#include "math.h"

const double MazebusterIR::_short_coeff[][5] = {
    {-3709.2846282640285, 3211.647120645552, -984.3284545311304, 129.74449784370552, -6.277161675805686},
    {18474.580338735428, -13043.42315339657, 3466.2528439112593, -409.7811596271419, 18.154027186496105},
    {248.06195234036207, 143.05386458809906, -100.91895379995788, 17.790035920182184, -1.0043657975841391},
    {-4554.180868917684, 3982.01736499226, -1227.559923331906, 162.25571778302603, -7.8535185416874045}
};

const double MazebusterIR::_long_coeff[][5] = {
    {271.2398476747508, -2.118998468865403, 0.007560100641692968, -0.00001241601680128958, 7.52235010113e-9},
    {253.31922885327901, -1.9238446803405251, 0.006730496339466588, -0.00001090205800728239, 6.56412163743e-9}
};

MazebusterIR::MazebusterIR(int irPin, int label, int avg, int tolerance) {
  
    _irPin=irPin;
    _label=label;
    _avg=avg;
    _tol=tolerance/100;
    
    analogReference(DEFAULT);
 
}


// When you initialize the library object on your sketch you have to pass all the above parameters:

// irPin is obviously the pin where the IR sensor is attached
// avg is the number of readings the library does
// tolerance indicates how similar a value has to be from the last value to be taken as valid. It should be a
//    value between 0 and 100, like a %. A value of 93 would mean that one value has to be, at least, 93% to the
//    previous value to be considered as valid.
// sensorModel is a int to differentiate the two sensor models this library currently supports:
//    1080 is the int for the GP2Y0A21Y and 20150 is the int for GP2Y0A02YK. The numbers reflect the
//    distance range they are designed for (in cm)
 


bool MazebusterIR::isShort(int label){
    int _short = label - 1;
    if (_short >= 0 && _short < 10){
        return true;
    }
    return false;
}

const double* MazebusterIR::labelToCoeff(bool isShort, int label){
    int _short = label - 1;
    int _long = _short % 10;

    if (isShort)
        return _short_coeff[_short];
    else
        return _long_coeff[_long];
}

double MazebusterIR::cm() {
    
    double raw = analogRead(_irPin);

    bool _isShort = isShort(_label);
    const double* _coeff = labelToCoeff(_isShort, _label);

    double distance = 0;

    if (_isShort)
        raw = log(raw);

    // Serial.print("Log: ");
    // Serial.println(raw);
    for(int i = 0; i < 5; i++){
        // Serial.print("Coeff: ");
        // Serial.println(_coeff[i]);
        distance += _coeff[i] * pow(raw, i);
        // Serial.print("Dist: ");
        // Serial.println(distance);
    }
    
    // Serial.println();

    return distance;
}



double MazebusterIR::distance() {

    _p=0;
    _sum=0;

    
    for (int i=0; i<_avg; i++){
        
        double foo=cm();
        
        if (foo>=(_tol*_previousDistance)){
        
            _previousDistance=foo;
            _sum=_sum+foo;
            _p++;
            
        }
        
        
    }

    
    double accurateDistance=_sum/_p;
    
    return accurateDistance;

}




