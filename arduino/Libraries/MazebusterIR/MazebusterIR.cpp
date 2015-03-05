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

// const double MazebusterIR::_short_coeff[][5] = {
//     {-3709.2846282640285, 3211.647120645552, -984.3284545311304, 129.74449784370552, -6.277161675805686},
//     {18474.580338735428, -13043.42315339657, 3466.2528439112593, -409.7811596271419, 18.154027186496105},
//     {248.06195234036207, 143.05386458809906, -100.91895379995788, 17.790035920182184, -1.0043657975841391},
//     {-4554.180868917684, 3982.01736499226, -1227.559923331906, 162.25571778302603, -7.8535185416874045}
// };

// const double MazebusterIR::_short_coeff[][8] = {
//     {8344.640556, -288.7060757, 4.168101232, -3.227048539e-2, 1.441491818e-4, -3.702362147e-7, 5.043816751e-10, -2.798817904e-13},
//     {1232.41807, -35.5203526, 4.516606564e-1, -3.142531234e-3, 1.276852788e-5, -3.011123151e-8, 3.800973631e-11, -1.975745298e-14},
//     {-1071.019322, 39.74187586, -5.673774756e-1, 4.245325137e-3, -1.814165886e-5, 4.439800066e-8, -5.764147097e-11, 3.058983002e-14},
//     {-3527.230194, 118.7442355, -1.617144026, 1.172934478e-2, -4.904149944e-5, 1.181275204e-7, -1.514595602e-10, 7.946425957e-14}
// };

const double MazebusterIR::_short_coeff[][2] = {
    {3.65150572,-9.696596938e-1},
    {3.880940635, -1.050268837},
    {3.606741049, -9.444125255e-1},
    {4.01641805, -1.106760198}
};

// 2573.771328 x5 - 30388.84174 x4 + 143063.2905 x3 - 335626.8516 x2 + 392259.3213 x - 182606.5791
// const double MazebusterIR::_short_coeff[][5] = {
//     {},
//     {},
//     {-182606.5791, 392259.3213, -335626.8516, 143063.2905, -30388.84174, 2573.771328},
//     {}
// };

const double MazebusterIR::_long_coeff[][5] = {
    {4.321618961, -1.097304877},
    {4.223062289, -1.059391527}
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

    raw = log10(raw);

    for(int i = 0; i < 2; i++){
        // Serial.print("Coeff: ");
        // Serial.println(_coeff[i]);
        distance += _coeff[i] * pow(raw, i);
        // Serial.print("Dist: ");
        // Serial.println(distance);
    }
    
    // Serial.println();

    return pow(10, distance);
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




