/************************************************************************************************************
 * SharpIR.h - Arduino library for retrieving distance (in cm) from the analog GP2Y0A21Y and GP2Y0A02YK     *
 * Distance sensors                                                                                         *
 * Copyright 2014 Dr. Marcal Casas-Cartagena (marcal.casas@gmail.com)                                       *
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

#ifndef MazebusterIR_h
#define MazebusterIR_h

#include "Arduino.h"

class MazebusterIR
{
  public:
    MazebusterIR (int irPin, int label, int avg, int tolerance);
    double distance();
    double cm();
    
  private:
    

    bool isShort(int label);

    const double* labelToCoeff(bool isShort, int label);

    static const double _short_coeff[][2];
    static const double _long_coeff[][5];
    
    int _irPin;
    int _label;
    int _avg;
    int _p;
    double _sum;
    double _previousDistance;
    int _tol;
    
};

#endif
