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

#include "AnchorPublisher.h"
#include <visualization_msgs/MarkerArray.h>


int main(int argc, char *argv[])
{

  ros::init(argc, argv, "anchors");
  ros::NodeHandle n;
  ros::Publisher pub_anchors = n.advertise<visualization_msgs::MarkerArray>("gtec/toa/anchors", 200);

  AnchorPublisher anAnchorPublisher;
  anAnchorPublisher.start("configBeacons", pub_anchors);

  ros::Rate loop_rate(5);

  while (ros::ok())
  {
    anAnchorPublisher.publishAnchors();
    ros::spinOnce();
    loop_rate.sleep();
  }


  return 0;
}
