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
#include "RangingDifference.h"


int main(int argc, char *argv[])
{


    ros::init(argc, argv, "rangingdifference");
    ros::NodeHandle n("~");
    ros::Publisher aPublisher = n.advertise<gtec_msgs::RangingDiff>("/gtec/toa/ranging/diff", 1000);


    std::string only = "";
    if (n.param<std::string>("paramOnly", only,"0,1,2,3")){
        ROS_INFO("only: %s",only.c_str());
    } else {
        ROS_INFO("Parameter error");
    }


    for (std::string::iterator it = only.begin(); it != only.end(); ++it) {
        if (*it == ',') {
            *it = ' ';
        }
        else continue;
    }


    std::vector<int> onlyAnchors;
    std::stringstream ss(only);
    int anchorId;

    while (ss >> anchorId) {
        onlyAnchors.push_back(anchorId);
    }


    RangingDifference rangingDifference(aPublisher, onlyAnchors);
    ros::Subscriber sub0 = n.subscribe<visualization_msgs::MarkerArray>("/gtec/toa/anchors", 2, &RangingDifference::newAnchorsMarkerArray, &rangingDifference);
    ros::Subscriber sub1 = n.subscribe<gtec_msgs::Ranging>("/gtec/toa/ranging", 12, &RangingDifference::newRanging, &rangingDifference);
    //ros::Subscriber sub2 = n.subscribe<geometry_msgs::PoseWithCovarianceStamped>("/gtec/uwb/pos", 12, &RangingDifference::newPos, &rangingDifference);
 ros::Subscriber sub2 = n.subscribe<geometry_msgs::PoseWithCovarianceStamped>("/fcu/pose", 12, &RangingDifference::newPos, &rangingDifference);


    ros::spin();

    return 0;
}
