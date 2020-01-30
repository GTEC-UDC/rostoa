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
#include "PozyxFixer.h"


PozyxFixer::PozyxFixer(ros::Publisher aPub, double cableLength) {
    this->cableLength = cableLength;
    this->ros_pub = aPub;
}

PozyxFixer::~PozyxFixer() {

}

void PozyxFixer::newRanging(const gtec_msgs::PozyxRanging::ConstPtr& pozyx_ranging_msg) {

    gtec_msgs::Ranging ranging;
    int raw_range;

    if ((pozyx_ranging_msg->originType == 0) && (pozyx_ranging_msg->destinationType == 1)) {
        //Origin es un tag
        //Destination es un anchor
        ranging.anchorId = pozyx_ranging_msg->destinationId;
        ranging.tagId = pozyx_ranging_msg->originId;
    } else {
        //Suponemos que siempre deberia corregirse medida de tag a anchor,
        //ya que el mensaje de ranging esta asi pensado
        ranging.anchorId = pozyx_ranging_msg->originId;
        ranging.tagId = pozyx_ranging_msg->destinationId;
    }


    ranging.range = pozyx_ranging_msg->range - (int)(this->cableLength * 1000);
    ranging.seq = pozyx_ranging_msg->seq;
    ranging.rss = pozyx_ranging_msg->rxPower;

    //TODO: hay que cambiar esto de alguna forma
    ranging.errorEstimation = 0.00393973;

    ros_pub.publish(ranging);
        //ROS_INFO("Pozyx Ranging Corrected:[AnchorId:%d, TagId:%d, Range:%d, ErrorEstimation:%f, SEQ:%d]", ranging.anchorId, ranging.tagId, ranging.range, ranging.errorEstimation, ranging.seq);

}
