/*
MIT License

Copyright (c) 2020 Group of Electronic Technology and Communications. University of A Coruna.

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/
#ifndef GTEC_BIAS_MANAGER_H
#define GTEC_BIAS_MANAGER_H


#include <string>
#include <vector>
#include <stdint.h>
#include <stdio.h>
#include <iostream>
#include <math.h>
#include <armadillo>
#include "ros/ros.h"
#include <sstream>

#define BIAS_MODE_LOS 0
#define BIAS_MODE_NLOS_FP 1
#define BIAS_MODE_NLOS_OP 2
#define BIAS_MODE_AUTO 3

class BiasManager
{

public:
     BiasManager();
    ~BiasManager();

    bool loadBias(int biasMode, std::string filenameTableBias,std::string filenameTableBiasError);
    double getBias(int biasMode, int ranging, double rxPower);
    double getBiasError(int biasMode, int ranging, double rxPower);
    double getRxPowerEstimation(int prf, int cir, int preambleCount);
    double getFirstPathPowerEstimation(int prf, int fp1, int fp2, int fp3, int preambleCount);
    
private:

    int getIndexPower(double rxPower, int minPowerValue, int maxPowerValue, double stepValue, int maxIndex);
    int getIndexRanging(int ranging, int minRanginValue, int maxRangingValue, int stepValue, int maxIndex);

    
    arma::mat biasLOS;
    arma::mat biasLOSError;
    arma::mat biasNLOSFP;
    arma::mat biasNLOSFPError;
    arma::mat biasNLOSOP;
    arma::mat biasNLOSOPError;
};


#endif
