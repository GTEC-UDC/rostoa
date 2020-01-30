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
#include <gtec_msgs/Ranging.h>
#include <gtec_msgs/PozyxRanging.h>

int main(int argc, char *argv[])
{


    ros::init(argc, argv, "pozyxfixer");
    ros::NodeHandle n("~");
    

    double cableLength;
    std::string topicRanging,topicOutputFixedRanging;

    n.getParam("cableLength", cableLength);
    n.getParam("topicRanging", topicRanging);
    n.getParam("topicOutputFixedRanging", topicOutputFixedRanging);


    ros::Publisher aPublisher = n.advertise<gtec_msgs::Ranging>(topicOutputFixedRanging, 1000);


    PozyxFixer pozyxFixer(aPublisher, cableLength);
    
    ros::Subscriber sub0 = n.subscribe<gtec_msgs::PozyxRanging>(topicRanging, 12, &PozyxFixer::newRanging, &pozyxFixer);

    ros::spin();

    return 0;
}
