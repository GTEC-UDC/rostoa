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

#include "GenericRangingFixer.h"





GenericRangingFixer::GenericRangingFixer(ros::Publisher aPub,bool useCustomBiasFix, int customBiasFixMode, double cableLength, BiasManager &biasManager) {
    this->useCustomBiasFix = useCustomBiasFix;
    this->customBiasFixMode = customBiasFixMode;
    this->cableLength = cableLength;
    this->ros_pub = aPub;
    this->biasManager = biasManager;
}

GenericRangingFixer::~GenericRangingFixer() {

}


void GenericRangingFixer::newGenericRanging(const gtec_msgs::GenericRanging::ConstPtr& generic_ranging_msg) {

    gtec_msgs::Ranging ranging;
    int raw_range;

    ranging.anchorId = generic_ranging_msg->anchorId;
    ranging.tagId = generic_ranging_msg->tagId;
    ranging.range = generic_ranging_msg->range - (int)(this->cableLength*1000);
    raw_range = generic_ranging_msg->range - (int)(this->cableLength*1000);
    ranging.seq = generic_ranging_msg->seq;

    if (this->useCustomBiasFix == true) {

        double rxPower = generic_ranging_msg->rxPower;

        //OJO: aqui no vale el bias auto, hay que escoger LOS o NLOS
        int selectedBias = this->customBiasFixMode; 

        double customBias = this->biasManager.getBias(selectedBias, raw_range, rxPower);
        ranging.errorEstimation = this->biasManager.getBiasError(selectedBias, raw_range, rxPower);

        int bias_correction = floor(customBias * 1000);
        ranging.range = raw_range - bias_correction;
    } else {
        //TODO: cambiar lo siguiente en algun momento
        ranging.errorEstimation = 0.00393973;
    }


    ros_pub.publish(ranging);
    //ROS_INFO("Generic Ranging Corrected:[AnchorId:%d, TagId:%d, Range:%d, ErrorEstimation:%f, SEQ:%d]", ranging.anchorId, ranging.tagId, ranging.range, ranging.errorEstimation, ranging.seq);
}
