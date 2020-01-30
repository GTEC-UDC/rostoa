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

#include "UWBFixer.h"
#include "BiasManager.h"
#include <gtec_msgs/Ranging.h>
#include <gtec_msgs/UWBRanging.h>

int main(int argc, char *argv[])
{


    ros::init(argc, argv, "uwbfixer");
    ros::NodeHandle n("~");
    ros::Publisher aPublisher = n.advertise<gtec_msgs::Ranging>("/gtec/toa/ranging", 1000);

    bool useCustomBiasFix;
    int customBiasFixMode;
    double cableLength;

    n.getParam("useCustomBiasFix", useCustomBiasFix);
    n.getParam("customBiasFixMode", customBiasFixMode);
    n.getParam("cableLength", cableLength);

    BiasManager biasManager;
    biasManager.loadBias(BIAS_MODE_LOS, "tableBiasLOS", "tableBiasLOSError");
    biasManager.loadBias(BIAS_MODE_NLOS_FP, "tableBiasNLOSFP", "tableBiasNLOSFPError");

    UWBFixer uwbFixer(aPublisher, useCustomBiasFix, customBiasFixMode, cableLength, biasManager);
    
    ros::Subscriber sub0 = n.subscribe<gtec_msgs::UWBRanging>("/gtec/uwb/ranging", 12, &UWBFixer::newUWBRanging, &uwbFixer);

    ros::spin();

    return 0;
}
