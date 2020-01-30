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

#include "NNFixer.h"
#include <gtec_msgs/Ranging.h>

int main(int argc, char *argv[])
{


    ros::init(argc, argv, "nnfixer");
    ros::NodeHandle n("~");
    

    int mode = 0;
    double minRange, maxRange, minRSS, maxRSS;

    std::string topicRanging, topicOutputFixedRanging;

    n.getParam("mode", mode);
    n.getParam("topicRanging", topicRanging);
    n.getParam("topicOutputFixedRanging", topicOutputFixedRanging);
    n.getParam("minRange", minRange);
    n.getParam("maxRange", maxRange);
    n.getParam("minRSS", minRSS);
    n.getParam("maxRSS", maxRSS);

    ros::Publisher aPublisher = n.advertise<gtec_msgs::Ranging>(topicOutputFixedRanging.c_str(), 1000);

    NNFixer nnFixer(aPublisher, mode, minRange, maxRange, minRSS, maxRSS);
    
    ros::Subscriber sub0 =n.subscribe<gtec_msgs::Ranging>(topicRanging, 12, &NNFixer::newRanging, &nnFixer);

    ros::spin();

    return 0;
}
