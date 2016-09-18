multilaser_surveillance
=======================

This package provides tools to perform surveillance on a known area.
The area is watched by a number of fixed 2D laser range finders.

Multimodal tracking is based on the
[perception stack of the STRANDS project](https://github.com/strands-project/strands_perception_people).
This stack makes use of
[```BayesTracking```](https://github.com/LCAS/bayestracking),
a library of Bayesian tracking.
For more info, read
*[Real-time multisensor people tracking for human-robot spatial interaction](http://eprints.lincoln.ac.uk/17545/)*
by Dondrup and Bellotto.

Steps:

1) ```MapBuilderWatcher```
  - Build the map based on the stream of laser scans.

2) ```MapBuilderWatcher```
  - Compare the streams of laser scans to the map and detect outliers w.r.t. the map

3) ```2dclusterer```
  - cluster outliers into continuous blobs, and publish their barycenter.

4) ```bayes_people_tracker```
  - convert the discontinuous blobs barycenters into tracks,
  using Unscented Kalman Filter.

Licence
=======

BSD


Authors
=======

  - Package maintainer: Arnaud Ramey (arnaud.a.ramey@gmail.com)
  - ```strands_perception_people```: [STRANDS project](http://strands.acin.tuwien.ac.at/)
  - ```BayesTracking``` library: Nicola Bellotto (nbellotto@lincoln.ac.uk)

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

Parameters
============

```map_builder_watcher``` - map builder:

  * `~frames [std_msgs/String]`, default ```""```
    Semi-colon-separated list of the frame of each 2D scan defined in ```~scan_topics```.

  * `~static_frame [std_msgs/String]`, default ```"/map"```
    The static frame for the map. The scans are converted into this frame.

  * `~mode [std_msgs/String]`, default ```"surveillance"```

  * `~map_prefix [std_msgs/String]`, default ```"mymap"```
    Where to save or load (according to the mode) the map file.
    Corresponds to a ```.csv``` and a ```.png``` file
    (these extensions are aggregated to the parameter value).

  * `~scan_topics [std_msgs/String]`, default ```""```
    Semi-colon-separated list of topics of 2D scans.


Publications
============

```map_builder_watcher``` - map builder:

  * `/map [nav_msgs/OccupancyGrid]`
    The map, shaped as an occupancy grid.

  * `/marker [visualization_msgs/Marker]`
    A marker showing the outliers and the clusters as colors.

  * `/scan [sensor_msgs/PointCloud]`
    The merged scan of all lasers, rate: max 1 Hz.

Clusterer:

  * `/cluster_centers [geometry_msgs/PoseArray]`
    In the outliers, the centers of the clusters.

  * `/outliers [sensor_msgs/PointCloud]`
    The points not corresponding to the map.

Tracker:

  * `/people_tracker/pose_array [geometry_msgs/PoseArray]`
    The pose of each object, obtained by Bayesian filtering.


Troubleshooting
===============

***Problem***:
The Bayesian tracker does not create tracks
if my detector framerate is below 5 Hz (200 ms).

***Explanation***:
By default, the [BayesTracking multitracker](https://github.com/LCAS/bayestracking/blob/dba55e38d59159d6d7a9ef70dd17909e4bdc3084/include/bayes_tracking/multitracker.h) creates tracks if it receives detections at least every 200 ms,
cf. constructor:

```cpp
MultiTracker(unsigned int sequenceSize = 5, double sequenceTime = 0.2)
```

And the embedded ```MultiTracker``` embedded in  [people_tracker/simple_tracking.h](https://github.com/strands-project/strands_perception_people/blob/ac2318f80ca8aeaa28c19a0393bdb0b39edd4a18/bayes_people_tracker/include/bayes_people_tracker/simple_tracking.h)
uses the default constructor:

```cpp
MultiTracker<FilterType, 4> mtrk; // state [x, v_x, y, v_y]
```

***Solution***:
change ```sequenceTime``` in MultiTracker instantiation.

Open
[people_tracker/simple_tracking.h](https://github.com/strands-project/strands_perception_people/blob/ac2318f80ca8aeaa28c19a0393bdb0b39edd4a18/bayes_people_tracker/include/bayes_people_tracker/simple_tracking.h)

and change the line

```cpp
SimpleTracking() {
```

for:

```cpp
SimpleTracking() : mtrk(5, .5) {
```
