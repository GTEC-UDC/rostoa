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

#include "BiasManager.h"





BiasManager::BiasManager() {

}

BiasManager::~BiasManager() {

}


double BiasManager::getRxPowerEstimation(int prf, int cir, int preambleCount) {
    float A = 113.77;

    if (prf == 64) {
        A = 121.74;
    }

    return  10 * log10(((cir * (pow(2, 17))) / pow(preambleCount, 2)) / pow(10, (A / 10)));
}


double BiasManager::getFirstPathPowerEstimation(int prf, int fp1, int fp2, int fp3, int preambleCount) {
    float A = 113.77;

    if (prf == 64) {
        A = 121.74;
    }

    return  10 * log10((((pow(fp1, 2)) + (pow(fp2, 2)) + (pow(fp3, 2))) / pow(preambleCount, 2)) / pow(10, (A / 10)));
}

//Carga desde un fichero las tablas de valores
bool BiasManager::loadBias(int biasMode, std::string filenameTableBias, std::string filenameTableBiasError) {

    ros::NodeHandle node_handle("~");
    std::string filenameTableBiasReal;
    std::string filenameTableBiasErrorReal;

    node_handle.getParam(filenameTableBias, filenameTableBiasReal);
    node_handle.getParam(filenameTableBiasError, filenameTableBiasErrorReal);

    switch (biasMode) {
    case BIAS_MODE_LOS:
        this->biasLOS.load(filenameTableBiasReal, arma::csv_ascii);
        this->biasLOSError.load(filenameTableBiasErrorReal, arma::csv_ascii);
        break;
    case BIAS_MODE_NLOS_FP:
        this->biasNLOSFP.load(filenameTableBiasReal, arma::csv_ascii);
        this->biasNLOSFPError.load(filenameTableBiasErrorReal, arma::csv_ascii);
        break;
    case BIAS_MODE_NLOS_OP:
        this->biasNLOSOP.load(filenameTableBiasReal, arma::csv_ascii);
        this->biasNLOSOPError.load(filenameTableBiasErrorReal, arma::csv_ascii);
        break;
    }

    return true;
}

int BiasManager::getIndexRanging(int ranging, int minRanginValue, int maxRangingValue, int stepValue, int maxIndex) {
    //TODO: meter los maximos como constantes o parametros
    int indexRanging = 0;
    if (ranging < minRanginValue) {
        indexRanging = 0;
    } else if (ranging > maxRangingValue) {
        indexRanging = maxIndex-1;
    } else {
        indexRanging = ranging / stepValue;
    }

    if (indexRanging>=maxIndex){
        indexRanging = maxIndex;
    } else if (indexRanging<0){
        indexRanging = 0;
    }

    return indexRanging;
}

int BiasManager::getIndexPower(double rxPower, int minPowerValue, int maxPowerValue, double stepValue, int maxIndex) {
    //TODO: meter los maximos como constantes o parametros
    int indexPower = 0;
    if (rxPower < minPowerValue) {
        indexPower = 0;
    } else if (rxPower > maxPowerValue) {
        indexPower = maxIndex-1;
    } else {
        indexPower = (rxPower + (-1 * minPowerValue)) / stepValue + 1;
    }

    if (indexPower>=maxIndex){
        indexPower = maxIndex;
    } else if (indexPower<0){
        indexPower = 0;
    }

    return indexPower;
}

//Devuelve una estimacion de bias para el ranging y potencia dados
// Estimacion en mm
double BiasManager::getBias(int biasMode, int ranging, double rxPower) {
    int indexRanging = 0;
    int indexPower = 0;

    switch (biasMode) {
    case BIAS_MODE_LOS:
        indexRanging = getIndexRanging(ranging, 0, 40000, 500, biasLOS.n_cols);
        indexPower = getIndexPower(rxPower, -105, -77, 0.5, biasLOS.n_rows);
        //std::cout << "IndexRanging: " << indexRanging << " IndexPower: " << indexPower << "\n"<<std::flush;
        //std::cout << "Ranging: " << ranging << " rxPower: " << rxPower << "\n"<<std::flush;
        return (biasLOS(indexPower, indexRanging));
        break;
    case BIAS_MODE_NLOS_FP:
//        indexRanging= getIndexRanging(ranging, biasNLOSFP.min(), biasNLOSFP.max(), 500, biasNLOSFP.n_cols);
//        indexPower = getIndexPower(rxPower, biasNLOSFP.min(), biasNLOSFP.max(), 0.5, biasNLOSFP.n_rows);

        indexRanging = getIndexRanging(ranging, 0, 40000, 500, biasLOS.n_cols);
        indexPower = getIndexPower(rxPower, -105, -77, 0.5, biasLOS.n_rows);
        return (biasNLOSFP(indexPower, indexRanging));
        break;
    case BIAS_MODE_NLOS_OP:
//        indexRanging= getIndexRanging(ranging, biasNLOSOP.min(), biasNLOSOP.max(), 500, biasNLOSOP.n_cols);
//        indexPower = getIndexPower(rxPower, biasNLOSOP.min(), biasNLOSOP.max(), 0.5, biasNLOSOP.n_rows);
        indexRanging = getIndexRanging(ranging, 0, 40000, 500, biasLOS.n_cols);
        indexPower = getIndexPower(rxPower, -105, -77, 0.5, biasLOS.n_rows);
        return (biasNLOSOP(indexPower, indexRanging));
        break;
    default:
        return 0;
    }
}

double BiasManager::getBiasError(int biasMode, int ranging, double rxPower) {
    int indexRanging = 0;
    int indexPower = 0;

    switch (biasMode) {
    case BIAS_MODE_LOS:
        indexRanging = getIndexRanging(ranging, 0, 40000, 500, biasLOSError.n_cols);
        indexPower = getIndexPower(rxPower, -105, -77, 0.5, biasLOSError.n_rows);
        //std::cout << "IndexRanging: " << indexRanging << " IndexPower: " << indexPower << "\n"<<std::flush;
        //std::cout << "Ranging: " << ranging << " rxPower: " << rxPower << "\n"<<std::flush;
        return (biasLOSError(indexPower, indexRanging));
        break;
    case BIAS_MODE_NLOS_FP:
        //indexRanging= getIndexRanging(ranging, biasNLOSFPError.min(), biasNLOSFPError.max(), 500, biasNLOSFPError.n_cols);
        //indexPower = getIndexPower(rxPower, biasNLOSFPError.min(), biasNLOSFPError.max(), 0.5, biasNLOSFPError.n_rows);
        indexRanging = getIndexRanging(ranging, 0, 40000, 500, biasLOSError.n_cols);
        indexPower = getIndexPower(rxPower, -105, -77, 0.5, biasLOSError.n_rows);
        return (biasNLOSFPError(indexPower, indexRanging));
        break;
    case BIAS_MODE_NLOS_OP:
        //indexRanging= getIndexRanging(ranging, biasNLOSOPError.min(), biasNLOSOPError.max(), 500, biasNLOSOPError.n_cols);
        // = getIndexPower(rxPower, biasNLOSOPError.min(), biasNLOSOPError.max(), 0.5, biasNLOSOPError.n_rows);
        indexRanging = getIndexRanging(ranging, 0, 40000, 500, biasLOSError.n_cols);
        indexPower = getIndexPower(rxPower, -105, -77, 0.5, biasLOSError.n_rows);
        return (biasNLOSOPError(indexPower, indexRanging));
        break;
    default:
        return 0;
    }
}
