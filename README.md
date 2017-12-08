multilaser_surveillance
=======================

[![Build Status](https://travis-ci.org/arnaud-ramey/multilaser_surveillance.svg)](https://travis-ci.org/arnaud-ramey/multilaser_surveillance)

This package provides tools to perform surveillance on a known area.
The area is watched by a number of fixed 2D laser range finders.

Multimodal tracking is based on the
[perception stack of the STRANDS project](https://github.com/strands-project/strands_perception_people).
This stack makes use of
[`BayesTracking`](https://github.com/LCAS/bayestracking),
a library of Bayesian tracking.
For more info, read
*[Real-time multisensor people tracking for human-robot spatial interaction](http://eprints.lincoln.ac.uk/17545/)*
by Dondrup and Bellotto.

Steps / pipeline:

1) `MapBuilderWatcher` - map building mode
  - Build the map based on the stream of laser scans.

2) `MapBuilderWatcher` - surveillance mode
  - Compare the streams of laser scans to the map and detect outliers w.r.t. the map

3) `2dclusterer`
  - cluster outliers into continuous blobs, and publish their barycenter.

4) `bayes_people_tracker`
  - convert the discontinuous blobs barycenters into tracks,
  using Unscented Kalman Filter.


Licence
=======

LGPLv3


Authors
=======

  - Package maintainer: Arnaud Ramey (arnaud.a.ramey@gmail.com)
  - `strands_perception_people`: [STRANDS project](http://strands.acin.tuwien.ac.at/)
  - `BayesTracking` library: Nicola Bellotto (nbellotto@lincoln.ac.uk)


Compile and install
===================

Dependencies handling is based on the [wstool](http://wiki.ros.org/wstool) tool.
Compile with [catkin_make](http://wiki.ros.org/catkin/commands/catkin_make):

```bash
$ sudo apt-get install python-wstool
$ roscd ; cd src
$ wstool init
$ wstool merge `rospack find multilaser_surveillance`/dependencies.rosinstall
$ wstool update
$ rosdep install multilaser_surveillance --ignore-src
$ catkin_make --only-pkg-with-deps multilaser_surveillance
```


Run
===

  1) Build the map.
  The created map is shown in `rviz` :
  it corresponds to the purple cells, obtained by accumulating the laser scans
  and inflating of a constant radius around them.
  The map is automatically saved in `multilaser_surveillance/data/maps`.

```bash
$ roslaunch multilaser_surveillance stage_arene.launch  mode:=build
```

  2) Perform surveillance.
  The map is loaded from the same folder.

```bash
$ roslaunch multilaser_surveillance stage_arene.launch
```


1) & 2) `MapBuilderWatcher` - map building & surveillance modes
===================================================================

Parameters
----------

  * `~apriori_map_service [string]`, default `""`.
    ONLY IN BUILD MODE
    The name of the service where an apriori map of the world can be found.
    It can be supplied using a [map_server](http://wiki.ros.org/map_server).
    If empty, won't be used, but in that case
    you need to supply `~xmin, ~ymin, ~xmax, ~ymax`, cf. below.

  * `~auto_mode_timeout [seconds, double]`, default 10.
    The time between switching automatically from map building to surveillance mode
    when in "auto" mode.

  * `~frames [string]`, default `""`.
    Semi-colon-separated list of the frame of each 2D scan defined in `~scan_topics`.
    Must be non empty and of the same size as `~scan_topics`.

  * `~static_frame [string]`, default `"/map"`.
    The static frame for the map. The scans are converted into this frame.

  * `~mode [string]`, default `"surveillance"`.
    Accepted values are `"auto", "build"` or `"surveillance"`.

  * `~map_prefix [string]`, default `"mymap"`.
    Where to save or load (according to the mode) the map file.
    Corresponds to a `.csv` and a `.png` file
    (these extensions are aggregated to the parameter value).

  * `~scan_topics [string]`, default `""`.
    Semi-colon-separated list of topics of 2D scans.
    Must be non empty and of the same size as `~scan_topics`.

  * `~xmin, ~ymin, ~xmax, ~ymax [double, meters]`, default -10,-10,10,10 meters.
    ONLY IN BUILD MODE, if `apriori_map_service` is empty or fails.
    Minimum and maximum values for the map boundaries.
    Scan values out of this range (in map coordinates) will be discarded.

Subscriptions
-------------

  * `$(scan_topics) [sensor_msgs/LaserScan]`
    The different scan streams published by the laser range finders.

  * `/tf [tf2_msgs/TFMessage]`
    The transforms between the frame of each scan and the `static_frame`.

Publications
------------

  * `~map [nav_msgs/OccupancyGrid]`
    The map, shaped as an occupancy grid.
    Rate: max 1 Hz.

  * `~marker [visualization_msgs/Marker]`
    A marker showing the outliers as a red point cloud,
    and the devices as arrows.
    Rate: max 1 Hz.

  * `~outliers [sensor_msgs/PointCloud]`
    ONLY IN SURVEILLANCE MDOE.
    The point cloud of all points not belonging to the map.
    Rate: upon reception of each scan, max 100 Hz.

  * `~scan [sensor_msgs/PointCloud]`
    The merged scan of all lasers.
    Rate: upon reception of each scan, max 100 Hz.


3) `2dclusterer`
====================

Parameters
----------

  * `~cluster_tolerance [double, meters]`, default 0.1 meter.
    The maximum distance between two points to consider them as belonging to the same cluster.

  * `~center_computation_method [std::string]`, default "fit".
    Accepted values: "fit" or "barycenter".
    Method to compute the center of each cluster,
    either by computing its barycenter,
    or by trying to best fit a circle of radius `~objects_radius`.

  * `~min_pts_per_cluster [unsigned int]`, default 1.
    The minimum of points in a cluster to keep it.
    Useful to avoid creating clusters with very few points.

  * `~max_clusters [unsigned int]`, default 20.
    The maximum number of clusters.
    Useful to avoid saturating the Bayesian tracker.

  * `~objects_radius [double, meters]`, default 0.5 meters.
    The radius of the objects to track.
    Only used in `center_computation_method = "fit"`.


Subscriptions
-------------

  * `~cloud [sensor_msgs/PointCloud]`
    The 2D point cloud to cluster, in `(x, y)`.

Publications
------------

  * `~cluster_centers [geometry_msgs/PoseArray]`
    The array containing the center of each cluster.
    For each center, the orientation is set to 0.

  * `~marker [visualization_msgs/Marker]`
    A marker showing the clusters as a colored point cloud,
    where the color corresponds to the cluster ID.
    Rate: upon reception of each PointCloud.


4) `bayes_people_tracker`
=============================

Parameters
----------

  * `detectors.yaml [YAML file]`
    A YAML file configuring the different detectors.
    See [STRANDS doc](https://github.com/strands-project/strands_perception_people/tree/indigo-devel/bayes_people_tracker)
    for more details.

Subscriptions
-------------

  * `~detectors/*/topic [geometry_msgs/PoseArray]`
    For each detector configured in the YAML file,
    the corresponding PoseArray topic.

Publications
------------

  * `/people_tracker/pose_array [geometry_msgs/PoseArray]`
    The pose of each object, obtained by Bayesian filtering.


Troubleshooting
---------------

***Problem***:
The Bayesian tracker does not create tracks
if my detector frame-rate is below 5 Hz (200 ms).

***Explanation***:
By default, the [BayesTracking multitracker](https://github.com/LCAS/bayestracking/blob/dba55e38d59159d6d7a9ef70dd17909e4bdc3084/include/bayes_tracking/multitracker.h)
creates tracks if it receives detections at least every 200 ms,
cf. function:

```cpp
line 160: void process(ObservationModelType& om, association_t alg = NN, unsigned int seqSize = 5, double seqTime = 0.2)
```

And the embedded `MultiTracker``` embedded in  [```people_tracker/simple_tracking.h`](https://github.com/strands-project/strands_perception_people/blob/ac2318f80ca8aeaa28c19a0393bdb0b39edd4a18/bayes_people_tracker/include/bayes_people_tracker/simple_tracking.h)
uses the default values for `seqSize` and `seqTime`.

***Solution***:
In MultiTracker class, add the parameters to the `process()` function call.
open
[`bayes_people_tracker/simple_tracking.h`](https://github.com/strands-project/strands_perception_people/blob/ac2318f80ca8aeaa28c19a0393bdb0b39edd4a18/bayes_people_tracker/include/bayes_people_tracker/simple_tracking.h)

and change the lines

```cpp
line 114: mtrk.process(*(it->second.ctm), it->second.alg);
line 158: mtrk.process(*(det.ctm), det.alg);
```

for:

```cpp
mtrk.process(*(it->second.ctm), it->second.alg, 5, .5);
mtrk.process(*(det.ctm), det.alg, 5, .5);
```

One-liner:

```bash
$ sed  -i 's/, it->second.alg/it, ->second.alg, 5, .5/g' `rospack find bayes_people_tracker`/include/bayes_people_tracker/simple_tracking.h
$ sed  -i 's/, det.alg/, det.alg, 5, .5/g' `rospack find bayes_people_tracker`/include/bayes_people_tracker/simple_tracking.h
$ catkin_make
```

pose_matcher
============


Parameters
----------

  * `~ground_truth_topics [string]`, default `""`.
    Semi-colon-separated list of topics of nav_msgs::Odometry.
    Must be non empty.

Subscriptions
-------------

  * `$(ground_truth_topics) [nav_msgs/Odometry]`
    The different ground truth streams,
    shaped as odometry messages and published by Stage for instance.

  * `~estimated_poses [geometry_msgs/PoseArray]`
    The estimated poses, generated by a tracker for instance.

Publications
------------

  * `~dist [std_msgs/Float32]`
    The average distance between the poses of `estimated_poses`
    and the different odometry streams.
