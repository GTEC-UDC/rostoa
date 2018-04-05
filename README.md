# README

This repository includes several tools and ROS nodes to process TOA (Time Of Arrival) measurements coming from [Decawave EVK1000/TRECK1000](https://decawave.com) or [Pozyx](https://www.pozyx.io) tags and anchors.

The nodes included in the repository are:

* **anchors** This node reads the 3D positions of a set of anchors from an .xml file and publish them in the ROS environment to be read by other nodes.
* **dwfixer** This node receives measurements from a Decawave EVK1000/TRECK1000 tag and applies a bias correction (See [Assessment of **UWB** Ranging Bias in Multipath Environments](http://www3.uah.es/ipin2016/usb/app/descargas/191_WIP.pdf) ). Finally it publishes the result in a new message of type gtec/Ranging.
* **pozyxfixer** This node receives measurements from a Pozyx tag and parses them to extract the information and to publish it as a new gtec/Ranging message. 

## Dependencies

GTEC ROS TOA package has the next dependencies:

* **gtec_msgs** Another GTEC ROS project with a set o custom messages definitions. Available here.
* **Boost** library. [http://www.boost.org](http://www.boost.org)
* **Armadillo** library [http://arma.sourceforge.net](http://arma.sourceforge.net)

## Building the nodes

To build the nodes, source code must be cloned inside a catkin work space on a ROS installation (see [Creating a workspace for catkin](http://wiki.ros.org/catkin/Tutorials/create_a_workspace)). If the catkin work space is located at ```~/catkin_ws``` then:

```bash
$ cd ~/catkin_ws/src
$ mkdir gtec
$ cd gtec
$ git clone rostoa
```

Then ```catkin_make``` must be used to build the nodes:

```bash
$ cd ~/catkin_ws
$ catkin_make
```

## Launching the nodes

There are several .launch files in the project to launch the nodes using the ```roslaunch``` command. 

Launching **anchors** node:

```bash
$ roslaunch gtec_rostoa anchors.launch
```

Launching **dwfixer** node:

```bash
$ roslaunch gtec_rostoa dwfixer.launch
```

Launching **pozyxfixer** node:

```bash
$ roslaunch gtec_rostoa pozyxfixer.launch
```

## Nodes configuration

Each node has different configuration parameters. Some of them are set through a .xml file while others are set in the .launch.

### Anchors node

There is an ```anchors.xml``` file inside ```~/catkin_ws/src/gtec/rostoa/config``` that defines the number and position of each anchor. The content of this file looks like:

```xml
<config>
 <anc ID="0" label="A0" x="2.129" y="6.114" z="1.652"/>
 <anc ID="1" label="A1" x="4.00" y="0.361" z="1.765"/>
 <anc ID="2" label="A2" x="7.327" y="6.478" z="1.752"/>
 <anc ID="3" label="A3" x="8.351" y="0.361" z="1.765"/>
 <anc ID="4" label="A4" x="12.513" y="2.886" z="1.834"/>
</config>
```

### DWFixer node

Parameters of DWFixer node are set in the ```~/catkin_ws/src/gtec/rostoa/dwfixer.launch``` file:

```xml
        <param name="useCustomBiasFix" type="bool" value="true" />
        <param name="customBiasFixMode" type="int" value="3" />
        <param name="cableLength" type="double" value="0.4" />
        <param name="tableBiasLOS" value="$(find gtec_rostoa)/src/rostoa/config/tableBiasLOS.csv"></param>
        <param name="tableBiasLOSError" value="$(find gtec_rostoa)/src/rostoa/config/tableBiasLOSError.csv"></param>
        <param name="tableBiasNLOSFP" value="$(find gtec_rostoa)/src/rostoa/config/tableBiasNLOSFP.csv"></param>
        <param name="tableBiasNLOSFPError" value="$(find gtec_rostoa)/src/rostoa/config/tableBiasNLOSFPError.csv"></param>
```

* **useCustomBiasFix**: if true the custom bias fix is applied.
* **customBiasFixMode**: different custom bias fix modes are available. Possible values are:
  * *0* Uses a scenario of LOS
  * *1* Uses a scenario of NLOS 
  * *3* Auto mode, switch from one scenario to other according to some quality parameters.
* **cableLength**: length in meter of the connection cable between the tag and the computer that receives the data. This length must be taken into account because it introduces a delay that must be corrected. 
* **tableBiasLOS, tableBiasLOSError, tableBiasNLOSFP, tableBiasNLOSFPError**: are a set of .csv files with real measurements of Decawave chips that are used by the node to correct the bias in the ranging estimation. 

### PozyxFixer

Parameters of PozyxFixer node are also set in the launch file located at ```~/catkin_ws/src/gtec/rostoa/pozyxfixer.launch```

```xml
<param name="numAnchors" type="int" value="6" />
<param name="cableLength" type="double" value="0.0" /
```

* **numAnchors**: the number of anchors to receive measurements from. Measurements from anchors with ids greater that this number will be ignored. 
* **cableLength**: length in meter of the connection cable between the tag and the computer that receives the data. This length must be taken into account because it introduces a delay that must be corrected. 