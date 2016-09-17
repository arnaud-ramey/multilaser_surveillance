multilaser_surveillance
=======================

Licence
=======
BSD


Authors
=======

Maintainer: Arnaud Ramey (arnaud.a.ramey@gmail.com)


Compile and install
===================

ROS Fuerte + rosmake
--------------------

Dependencies with ROS Fuerte:

```bash
$ sudo apt-get install  ros-fuerte-perception
```

Compile with rosmake:

```bash
$ cd cmake ; bash package2rosmake.bash
$ rosmake multilaser_surveillance
```

ROS Indigo + catkin
-------------------

Compile with [catkin_make](http://wiki.ros.org/catkin/commands/catkin_make):

```bash
$ roscd ; cd src
$ git clone https://github.com/strands-project/strands_perception_people.git
$ git clone https://github.com/LCAS/bayestracking.git
$ git clone https://github.com/wg-perception/people.git
$ rospack profile
$ catkin_make --only-pkg-with-deps multilaser_surveillance
```

Run
===

  1) Build the map.
    The map is automatically saved in `multilaser_surveillance/data/maps`.

```bash
$ roslaunch multilaser_surveillance stage_arenes.launch  mode:=build
```

  2) Perform surveillance.
    The map is loaded from the same folder.

```bash
$ roslaunch multilaser_surveillance stage_arenes.launch
```

Publications
============

  * `/cluster_centers [geometry_msgs/PoseArray]`
    In the outliers, the centers of the clusters.

  * `/map [nav_msgs/OccupancyGrid]`
    The map, shaped as an occupancy grid.

  * `/marker [visualization_msgs/Marker]`
    A marker showing the outliers and the clusters as colors.

  * `/outliers [sensor_msgs/PointCloud]`
    The points not corresponding to the map.

  * `/people_tracker/pose_array [geometry_msgs/PoseArray]`
    The pose of each object, obtained by Bayesian filtering.

  * `/scan [sensor_msgs/PointCloud]`
    The merged scan of all lasers, rate: max 1 Hz.

Troubleshooting
===============

