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
#ifndef gtec_rostoa_ANCHOR_PUBLISH_H
#define gtec_rostoa_ANCHOR_PUBLISH_H


#define BOOST_NO_CXX11_SCOPED_ENUMS
#include <boost/asio.hpp>
#include <boost/asio/serial_port.hpp>
#include <boost/system/error_code.hpp>
#include <boost/system/system_error.hpp>
#include <boost/bind.hpp>
#include <boost/thread.hpp>
#include <boost/assert.hpp>
#include <boost/property_tree/xml_parser.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/foreach.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/date_time/gregorian/gregorian.hpp>

#include <string>
#include <vector>
#include <stdint.h>
#include <stdio.h>
#include <iostream>
#include <math.h>
#include <chrono>
#include <armadillo>


#include "ros/ros.h"
#include <sstream>

#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>

#define MAX_NUM_ANCS (64)

typedef struct
{
    double x, y, z;
    uint64_t id;
    std::string label;
    int tagRangeCorection[64];
} anc_struct_t;


typedef struct
{
    double x;
    double y;
} vec2d;


struct Vector3 {
    double x, y, z;
    double rotX, rotY, rotZ, rotW;
    arma::mat covarianceMatrix;
};

struct Beacon {
    int id;
    Vector3 position;
};

class AnchorPublisher
{


public:
    AnchorPublisher();

    void start(std::string configFilenameBeacons,ros::Publisher anAnchorsPub);
    bool init(std::string filenameBeacons);
    void publishAnchors();


private:
    ros::Publisher ros_pub_anchors;
    anc_struct_t _ancArray[64];
    std::vector<Beacon> beacons;

};

#endif //gtec_rostoa_ANCHOR_PUBLISH_H
