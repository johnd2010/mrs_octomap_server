<!--
<div align="center">
    <h1>OctoMap Server2</h1>
    <h3>Implementation of octomap for ROS2.0 </h3>
    <a href="https://travis-ci.com/iKrishneel/octomap_server2"><img src="https://travis-ci.com/iKrishneel/octomap_server2.svg?branch=master"></a>
</div>
-->

#### Installation

Firstly make sure you have [octomap](https://github.com/OctoMap/octomap.git) installed on your system 

```bash
sudo apt install ros-noetic-octomap ros-noetic-octomap-server
```

Clone the repository to the workspace and build with catkin

#### Running

Launch the node with appropriate input on topic `cloud_in` or `laser_scan_in`
Make sure, that the correct fixed frame set in `config/params.yaml`

```bash
$ roslaunch octomap_server octomap_server.launch
```
