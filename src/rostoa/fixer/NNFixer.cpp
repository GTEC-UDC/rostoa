/*
MIT License

Copyright (c) 2018 Group of Electronic Technology and Communications. University of A Coruña.

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
#include "NNFixer.h"
#include "nnClassifier.h"
#include "nnClassifier_terminate.h"
#include "nnClassifier_initialize.h"

NNFixer::NNFixer(ros::Publisher aPub, int mode, double minRange, double maxRange, double minRSS, double maxRSS) {
    this->_mode = mode;
    this->ros_pub = aPub;
    this->_minRange = minRange;
    this->_maxRange = maxRange;
    this->_minRSS = minRSS;
    this->_maxRSS = maxRSS;
     nnClassifier_initialize();
}

NNFixer::~NNFixer() {
     nnClassifier_terminate();
}

double NNFixer::normalize(double value, double min, double max){
    return (value - min) / ( max - min );
}

void NNFixer::newRanging(const gtec_msgs::Ranging::ConstPtr& ranging_msg) {
    gtec_msgs::Ranging outputRanging;
    outputRanging.anchorId = ranging_msg->anchorId;
    outputRanging.tagId = ranging_msg->tagId;
    outputRanging.range = ranging_msg->range;
    outputRanging.seq = ranging_msg->seq;
    outputRanging.rss = ranging_msg->rss;
    outputRanging.errorEstimation = ranging_msg ->errorEstimation;

    double rangeNormalized = normalize(ranging_msg->range, _minRange, _maxRange);
    double rssNormalized = normalize(ranging_msg->rss, _minRSS, _maxRSS);

    int isLOS = nnClassifier(rssNormalized, rangeNormalized);

    if (_mode==0){
        //No ignore
        ros_pub.publish(outputRanging);
    } else if (_mode==1){
        //Ignore NLOS, NO mitigation
        if (isLOS==0){
            ros_pub.publish(outputRanging);
        } else {
            ROS_INFO("<<<<<<<Range ignored>>>>>>>>>>> AnchorId:%x", outputRanging.anchorId);
        }
    }
    //ROS_INFO("Pozyx Ranging Corrected:[AnchorId:%x, TagId:%x, Range:%d, ErrorEstimation:%f, SEQ:%d, isLOS: %d]", outputRanging.anchorId, outputRanging.tagId, outputRanging.range, outputRanging.errorEstimation, outputRanging.seq, isLOS);
    //ROS_INFO("Pozyx Ranging Corrected:[Range Normalized: %f rssNormalized: %f]", rangeNormalized, rssNormalized);

}
