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
#include <algorithm>

RangingDifference::RangingDifference(ros::Publisher aPub, std::vector<int>& anchorsToPublish) {

  this->mAnchorsSet = false;
  this->ros_pub = aPub;
  this->mLastPostSet = false;
  this->anchorsToPublish = anchorsToPublish;

  for (int i = 0; i < this->anchorsToPublish.size(); ++i)
  {
    ROS_INFO("Filter: %d",this->anchorsToPublish[i]);
  }
}

RangingDifference::~RangingDifference() {

}


void RangingDifference::newAnchorsMarkerArray(const visualization_msgs::MarkerArray::ConstPtr& anchorsMarkerArray) {

  if (!mAnchorsSet) {
    ROS_INFO("Setting anchors");
    for (int i = 0; i < anchorsMarkerArray->markers.size(); ++i)
    {

      visualization_msgs::Marker marker = anchorsMarkerArray->markers[i];
      mAnchorPositions.push_back({ i, marker.pose.position.x, marker.pose.position.y, marker.pose.position.z });
    }


    mAnchorsSet = true;
    ROS_INFO("Anchors set");

  }

}


void RangingDifference::newRanging(const gtec_msgs::Ranging::ConstPtr& ranging_msg) {

  if (mAnchorsSet && mLastPostSet) {
    float realDistance = getDistanceToLastPost(ranging_msg->anchorId);
    float ranging = ranging_msg->range / 1000.0;

    float diff = realDistance - ranging;

    gtec_msgs::RangingDiff msg;

    msg.anchorId = ranging_msg->anchorId;
    msg.tagId = ranging_msg->tagId;
    msg.range = ranging;
    msg.distance = realDistance;
    msg.diff = diff;


    if (std::find(anchorsToPublish.begin(), anchorsToPublish.end(), ranging_msg->anchorId) != anchorsToPublish.end()) {
      ros_pub.publish(msg);
    }


  }


}


float RangingDifference::getDistanceToLastPost(int anchorId) {
  if (mAnchorsSet && mLastPostSet) {

    return sqrt(pow(mAnchorPositions[anchorId].x - mLastPos.pose.pose.position.x, 2) +
                pow(mAnchorPositions[anchorId].y - mLastPos.pose.pose.position.y, 2) +
                pow(mAnchorPositions[anchorId].z - mLastPos.pose.pose.position.z, 2));
  } else {
    return 0;
  }

}


void RangingDifference::newPos(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& pos_msg) {
  if (mAnchorsSet) {

    mLastPos = *pos_msg;
    mLastPostSet = true;

  }

}


