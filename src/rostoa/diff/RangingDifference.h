#ifndef GTEC_TOA_RANGING_DIFFERENCE_H
#define GTEC_TOA_RANGING_DIFFERENCE_H


#include <string>
#include <vector>
#include <stdint.h>
#include <stdio.h>
#include <iostream>
#include <math.h>
#include <armadillo>

#include "ros/ros.h"
#include <sstream>

#include <gtec_msgs/Ranging.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <visualization_msgs/MarkerArray.h>
#include <gtec_msgs/RangingDiff.h>



struct AnchorPos
{
	int id;
    double x, y, z;
};


class RangingDifference
{

public:
     RangingDifference(ros::Publisher aPub, std::vector<int>& anchorsToPublish);
    ~RangingDifference();

    void newRanging(const gtec_msgs::Ranging::ConstPtr& ranging_msg);
    void newPos(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& pos_msg);
    void newAnchorsMarkerArray(const visualization_msgs::MarkerArray::ConstPtr& anchorsMarkerArray);
   

private:
	ros::Publisher ros_pub;
	std::vector<AnchorPos> mAnchorPositions;
    bool mAnchorsSet;
    bool mLastPostSet;
    float getDistanceToLastPost(int anchorId);
    geometry_msgs::PoseWithCovarianceStamped mLastPos;
    std::vector<int> anchorsToPublish;
};


#endif