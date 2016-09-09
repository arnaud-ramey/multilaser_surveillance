/*!
  \file         2dclusterer.cpp
  \author       Arnaud Ramey <arnaud.a.ramey@gmail.com>
                -- Robotics Lab, University Carlos III of Madrid
  \date         2016/09/09

  ______________________________________________________________________________

  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU Lesser General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>.
  ______________________________________________________________________________
*/
#include <multilaser_surveillance/2dclusterer.h>
#include <multilaser_surveillance/timer.h>
// ROS
#include <tf/transform_listener.h>
// ROS msg
#include <sensor_msgs/PointCloud.h>
#include <geometry_msgs/PoseArray.h>

typedef geometry_msgs::Point32 Pt2;

class ROSClusterer {
public:
  ROSClusterer() : _nh_private("~") {
    _nh_private.param("cluster_tolerance", _cluster_tolerance, .1);
    _cloud_sub = _nh_public.subscribe<sensor_msgs::PointCloud>
        ("cloud", 0, &ROSClusterer::cloud_cb, this);
    _cluster_centers_pub = _nh_public.advertise<geometry_msgs::PoseArray>( "cluster_centers", 0 );
  }

  void cloud_cb(const sensor_msgs::PointCloud::ConstPtr& cloud_msg) {
    // DEBUG_PRINT("cloud_cb(%i)\n", device_idx);
    if (_cluster_centers_pub.getNumSubscribers() == 0)
      return;
    Timer timer;
    cluster(cloud_msg->points, _cluster_indices, _nclusters, _cluster_tolerance);
    barycenters(cloud_msg->points, _cluster_indices, _nclusters, _cluster_centers);
    // publish _cluster_centers_msg
    _cluster_centers_msg.header = cloud_msg->header;
    _cluster_centers_msg.poses.resize(_nclusters);
    for (unsigned int ci = 0; ci < _nclusters; ++ci) {
      _cluster_centers_msg.poses[ci].position.x = _cluster_centers[ci].x;
      _cluster_centers_msg.poses[ci].position.y = _cluster_centers[ci].y;
      _cluster_centers_msg.poses[ci].orientation.z = 1;
    } // end for ci
    _cluster_centers_pub.publish(_cluster_centers_msg);
    ROS_INFO_THROTTLE(5, "time for cloud_cb(%g): %g ms",
                      _cluster_tolerance, timer.getTimeMilliseconds());
  } // end cloud_cb()


  ros::NodeHandle _nh_public, _nh_private;
  ros::Subscriber _cloud_sub;
  ros::Publisher _cluster_centers_pub;
  double _cluster_tolerance;
  std::vector<unsigned int> _cluster_indices;
  unsigned int _nclusters;
  std::vector<Pt2> _cluster_centers;
  geometry_msgs::PoseArray _cluster_centers_msg;
}; // end class ROSClusterer

int main(int argc, char **argv) {
  ros::init(argc, argv, "2dclusterer");
  ROSClusterer clusterer;
  ros::spin();
  return 0;
}
