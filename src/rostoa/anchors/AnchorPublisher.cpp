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



AnchorPublisher::AnchorPublisher() {

}


void AnchorPublisher::start(std::string configFilenameBeacons, ros::Publisher anAnchorsPub) {
  //Cargamos la configuracion
  bool configLoaded = init(configFilenameBeacons);
  ros_pub_anchors = anAnchorsPub;
}


bool AnchorPublisher::init(std::string filenameBeacons) {

  try {

    //************************************
    //Cargamos la posicion de las balizas
    //************************************

    //ROS_INFO("POSGEN: Intentando cargar propiedad %s", filenameBeacons.c_str());
    boost::property_tree::ptree configTree;
    //ros::NodeHandle node_handle = new ros::NodeHandle("~");
    ros::NodeHandle node_handle("~");
    std::string configContent;
    node_handle.getParam(filenameBeacons, configContent);
    std::stringstream ss;
    ss << configContent;


    boost::property_tree::read_xml(ss, configTree);

    double x[64] = {0.0, 5.0, 0.0, 5.0};
    double y[64] = {0.0, 0.0, 5.0, 5.0};

    for (int i = 0; i < 64; i++) {
      _ancArray[i].id = 0xff;
      _ancArray[i].label = "";
      _ancArray[i].x = x[i]; //default x
      _ancArray[i].y = y[i]; //default y
      _ancArray[i].z = 3.00; //default z

      for (int j = 0; j < 64; j++) {
        _ancArray[i].tagRangeCorection[j] = 0;
      }
    }

    std::stringstream ssi;

    ROS_INFO("Reading configuration file");
     int index = 0;


    BOOST_FOREACH(const boost::property_tree::ptree::value_type & v, configTree.get_child("config")) {

      if (v.first.compare("anc") == 0) {
        std::string idText = v.second.get<std::string>("<xmlattr>.ID", "");

        int id;
        
        ssi << std::hex << idText.c_str();
        ssi >> id;

        ssi.str("");
        ssi.clear();

          //id &= 0x3;
         // _ancArray[id].id = id & 0xf;
          _ancArray[index].id = id;
          _ancArray[index].label = v.second.get<std::string>("<xmlattr>.label", "");


          _ancArray[index].x = v.second.get<double>("<xmlattr>.x", 0.0);
          _ancArray[index].y = v.second.get<double>("<xmlattr>.y", 0.0);
          _ancArray[index].z = v.second.get<double>("<xmlattr>.z", 0.0);


          std::cout << "Anchor " << id << ": (" << _ancArray[index].x << ", " << _ancArray[index].y <<", " << _ancArray[index].z << ")\n" << std::flush;

          //tag distance correction (in cm)
/*          _ancArray[id].tagRangeCorection[0] = v.second.get<double>("<xmlattr>.t0", 0);
          _ancArray[id].tagRangeCorection[1] = v.second.get<double>("<xmlattr>.t1", 0);
          _ancArray[id].tagRangeCorection[2] = v.second.get<double>("<xmlattr>.t2", 0);
          _ancArray[id].tagRangeCorection[3] = v.second.get<double>("<xmlattr>.t3", 0);
          _ancArray[id].tagRangeCorection[4] = v.second.get<double>("<xmlattr>.t4", 0);
          _ancArray[id].tagRangeCorection[5] = v.second.get<double>("<xmlattr>.t5", 0);
          _ancArray[id].tagRangeCorection[6] = v.second.get<double>("<xmlattr>.t6", 0);
          _ancArray[id].tagRangeCorection[7] = v.second.get<double>("<xmlattr>.t7", 0);*/


          _ancArray[index].tagRangeCorection[0] = 0;
          _ancArray[index].tagRangeCorection[1] = 0;
          _ancArray[index].tagRangeCorection[2] = 0;
          _ancArray[index].tagRangeCorection[3] = 0;
          _ancArray[index].tagRangeCorection[4] = 0;
          _ancArray[index].tagRangeCorection[5] = 0;
          _ancArray[index].tagRangeCorection[6] = 0;
          _ancArray[index].tagRangeCorection[7] = 0;

        beacons.push_back({ id, { _ancArray[index].x, _ancArray[index].y, _ancArray[index].z } });
        index+=1;

      }
    }

    ROS_INFO("POSGEN: Beacons Config loaded");
    return true;

  } catch (boost::exception const &ex) {
    return false;
  }

  return false;
}


void AnchorPublisher::publishAnchors() {
  visualization_msgs::MarkerArray markerArray;
  for (int i = 0; i < beacons.size(); ++i)
  {
    visualization_msgs::Marker marker;
    marker.header.frame_id = "world";
    marker.header.stamp = ros::Time();
    marker.id = beacons[i].id;
    marker.type = visualization_msgs::Marker::CYLINDER;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = beacons[i].position.x;
    marker.pose.position.y = beacons[i].position.y;
    marker.pose.position.z = beacons[i].position.z;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.2;
    marker.scale.y = 0.2;
    marker.scale.z = 0.5;
    marker.color.a = 1.0; // Don't forget to set the alpha!

     if (i == 0) {
      marker.color.r = 0.6;
      marker.color.g = 0.0;
      marker.color.b = 0.0;
    } else if (i == 1) {
      marker.color.r = 0.6;
      marker.color.g = 0.2;
      marker.color.b = 0.0;
    } else if (i == 2) {
      marker.color.r = 0.6;
      marker.color.g = 0.5;
      marker.color.b = 1.0;
    } else if (i == 3) {
      marker.color.r = 0.6;
      marker.color.g = 0.8;
      marker.color.b = 0.0;
    }else if (i == 4) {
      marker.color.b = 0.6;
      marker.color.g = 0.0;
      marker.color.r = 0.0;
    } else if (i == 5) {
      marker.color.b = 0.6;
      marker.color.g = 0.2;
      marker.color.r = 0.0;
    } else if (i == 6) {
      marker.color.b = 0.6;
      marker.color.g = 0.5;
      marker.color.r = 1.0;
    } else if (i == 7) {
      marker.color.b = 0.6;
      marker.color.g = 0.8;
      marker.color.r = 0.0;
    }

    markerArray.markers.push_back(marker);
  }
  ros_pub_anchors.publish(markerArray);
}
