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

#include "PozyxNNFixer.h"
#include <gtec_msgs/Ranging.h>
#include <gtec_msgs/PozyxRanging.h>

int main(int argc, char *argv[])
{


    ros::init(argc, argv, "pozyxnnfixer");
    ros::NodeHandle n("~");
    ros::Publisher aPublisher = n.advertise<gtec_msgs::Ranging>("/gtec/toa/ranging", 1000);

    int mode = 0;
    int numAnchors = -1;
    n.getParam("mode", mode);
    n.getParam("numAnchors", numAnchors);

    PozyxNNFixer pozyxNNFixer(aPublisher, mode, numAnchors);
    
    ros::Subscriber sub0 = n.subscribe<gtec_msgs::PozyxRanging>("/gtec/uwb/ranging/pozyx", 12, &PozyxNNFixer::newRanging, &pozyxNNFixer);

    ros::spin();

    return 0;
}