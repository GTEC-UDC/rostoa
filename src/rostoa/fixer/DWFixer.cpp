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

#include "DWFixer.h"





DWFixer::DWFixer(ros::Publisher aPub,bool useCustomBiasFix, int customBiasFixMode, double cableLength, BiasManager &biasManager) {
    this->useCustomBiasFix = useCustomBiasFix;
    this->customBiasFixMode = customBiasFixMode;
    this->cableLength = cableLength;
    this->ros_pub = aPub;
    this->biasManager = biasManager;

}

DWFixer::~DWFixer() {

}

void DWFixer::newDWRanging(const gtec_msgs::DWRanging::ConstPtr& dw_ranging_msg) {
    gtec_msgs::Ranging ranging;
    int raw_range;

    ranging.anchorId = dw_ranging_msg->anchorId;
    ranging.tagId = dw_ranging_msg->tagId;
    ranging.range = dw_ranging_msg->range - (int)(this->cableLength*1000);
    raw_range = dw_ranging_msg->rawrange - (int)(this->cableLength*1000);
    ranging.seq = dw_ranging_msg->seq;

    if ((this->useCustomBiasFix == true) && (dw_ranging_msg->maxGrowthCIR>0)) {

        double rxPower = this->biasManager.getRxPowerEstimation(dw_ranging_msg->prf, dw_ranging_msg->maxGrowthCIR, dw_ranging_msg->rxPreamCount);
        double firstPathPower = this->biasManager.getFirstPathPowerEstimation(dw_ranging_msg->prf, dw_ranging_msg->firstPathAmp1, dw_ranging_msg->firstPathAmp2, dw_ranging_msg->firstPathAmp3, dw_ranging_msg->rxPreamCount);

        int selectedBias = BIAS_MODE_LOS;

        if (this->customBiasFixMode==BIAS_MODE_AUTO){
            double powerDiff = rxPower - firstPathPower;
            double losNlosThreshold = 7;
            if (powerDiff > losNlosThreshold) {
                selectedBias = BIAS_MODE_NLOS_FP;
            }
        } else {
            selectedBias = this->customBiasFixMode; 
        }

        double customBias = this->biasManager.getBias(selectedBias, raw_range, rxPower);
        ranging.errorEstimation = this->biasManager.getBiasError(selectedBias, raw_range, rxPower);

        int bias_correction = floor(customBias * 1000);
        ranging.range = raw_range - bias_correction;
    } else {
        //TODO: cambiar lo siguiente en algun momento
        ranging.errorEstimation = 0.00393973;
    }


    ros_pub.publish(ranging);
    ROS_INFO("UWB Ranging Corrected:[AnchorId:%d, TagId:%d, Range:%d, ErrorEstimation:%f, SEQ:%d]", ranging.anchorId, ranging.tagId, ranging.range, ranging.errorEstimation, ranging.seq);
}

