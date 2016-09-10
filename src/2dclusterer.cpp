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
#include <visualization_msgs/Marker.h>

typedef geometry_msgs::Point32 Pt2;

template<class Pt2a, class Pt2b>
void copy_vec(const std::vector<Pt2a> & src, std::vector<Pt2b> & dst) {
  unsigned int npts = src.size();
  dst.resize(npts);
  for (unsigned int i = 0; i < npts; ++i) {
    dst[i].x = src[i].x;
    dst[i].y = src[i].y;
  } // end for i
}

////////////////////////////////////////////////////////////////////////////////

static inline void createColorMsg
(std_msgs::ColorRGBA & color, const float & red, const float & green,
 const float & blue, const float & alpha = 1) {
  color.r = red;
  color.g = green;
  color.b = blue;
  color.a = alpha;
}

static const int NB_PREDEFINED_COLORS = 24;

////////////////////////////////////////////////////////////////////////////////

template<class _T>
inline void _indexed_color_templated
(_T & r, _T & g, _T & b,
 const int & index,
 const _T & MAX, const _T & HIGH, const _T & MED, const _T & LOW, const _T & ZERO) {

  switch (index % NB_PREDEFINED_COLORS) {
    case 0:  r = MAX;  g = ZERO; b = ZERO; break;
    case 1:  r = ZERO; g = MAX;  b = ZERO; break;
    case 2:  r = ZERO; g = ZERO; b = MAX; break;
    case 3:  r = MAX;  g = MAX;  b = ZERO; break;
    case 4:  r = MAX;  g = ZERO; b = MAX; break;
    case 5:  r = ZERO; g = MAX;  b = MAX; break;

    case 6:  r = MAX; g = MED; b = ZERO; break;
    case 7:  r = MED; g = MAX; b = ZERO; break;
    case 8:  r = MED; g = ZERO; b = MAX; break;
    case 9:  r = MAX; g = ZERO; b = MED; break;
    case 10: r = ZERO; g = MED; b = MAX; break;
    case 11: r = ZERO; g = MAX; b = MED; break;

    case 12: r = HIGH; g = ZERO; b = ZERO; break;
    case 13: r = ZERO; g = HIGH; b = ZERO; break;
    case 14: r = ZERO; g = ZERO; b = HIGH; break;

    case 15: r = HIGH; g = HIGH; b = ZERO; break;
    case 16: r = HIGH; g = ZERO; b = HIGH; break;
    case 17: r = ZERO; g = HIGH; b = HIGH; break;

    case 18: r = HIGH; g =  LOW; b = ZERO; break;
    case 19: r =  LOW; g = HIGH; b = ZERO; break;
    case 20: r =  LOW; g = ZERO; b = HIGH; break;
    case 21: r = HIGH; g = ZERO; b =  LOW; break;
    case 22: r = ZERO; g =  LOW; b = HIGH; break;
    default:
    case 23: r = ZERO; g = HIGH; b = LOW; break;
  } // end switch
} // end _indexed_color_templated

////////////////////////////////////////////////////////////////////////////////

inline void indexed_color_norm(float & r, float & g, float & b,
                               const int index = rand()) {
  _indexed_color_templated<float>(r, g, b, index, 1, 0.7, 0.5, 0.33, 0);
}

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

class ROSClusterer {
public:
  ROSClusterer() : _nh_private("~") {
    _nh_private.param("cluster_tolerance", _cluster_tolerance, .1);
    _cloud_sub = _nh_public.subscribe<sensor_msgs::PointCloud>
        ("cloud", 0, &ROSClusterer::cloud_cb, this);
    _cluster_centers_pub = _nh_public.advertise<geometry_msgs::PoseArray>( "cluster_centers", 0 );
    _marker_pub = _nh_public.advertise<visualization_msgs::Marker>( "marker", 0 );
  }

  void cloud_cb(const sensor_msgs::PointCloud::ConstPtr& cloud_msg) {
    // DEBUG_PRINT("cloud_cb(%i)\n", device_idx);
    if (_cluster_centers_pub.getNumSubscribers() == 0)
      return;
    Timer timer;
    cluster(cloud_msg->points, _cluster_indices, _nclusters, _cluster_tolerance);
    barycenters(cloud_msg->points, _cluster_indices, _nclusters, _cluster_centers);
    //printf("nclusters:%i\n", _nclusters);

    // publish _cluster_centers_msg
    _cluster_centers_msg.header = cloud_msg->header;
    _cluster_centers_msg.poses.resize(_nclusters);
    for (unsigned int ci = 0; ci < _nclusters; ++ci) {
      _cluster_centers_msg.poses[ci].position.x = _cluster_centers[ci].x;
      _cluster_centers_msg.poses[ci].position.y = _cluster_centers[ci].y;
      _cluster_centers_msg.poses[ci].position.z = 0;
      _cluster_centers_msg.poses[ci].orientation.z = 1;
    } // end for ci
    _cluster_centers_pub.publish(_cluster_centers_msg);

    // publish markers
    _marker_msg.header = cloud_msg->header;
    _marker_msg.type = visualization_msgs::Marker::POINTS;
    _marker_msg.action = visualization_msgs::Marker::ADD;
    _marker_msg.ns = "clusters";
    _marker_msg.id = 0; // unique identifier
    _marker_msg.scale.x = 0.2;
    _marker_msg.scale.y = 0.2;
    _marker_msg.scale.z = 0.2;
    copy_vec(cloud_msg->points, _marker_msg.points);
    unsigned int npts = cloud_msg->points.size();
    _marker_msg.colors.resize(npts);
    for (unsigned int i = 0; i < npts; ++i) {
      _marker_msg.points[i].z = 0.5; // a bit higher for readability
      _marker_msg.colors[i].a = 1; // no transparency
      indexed_color_norm(_marker_msg.colors[i].r, _marker_msg.colors[i].g, _marker_msg.colors[i].b,
                         _cluster_indices[i]);
    }
    _marker_pub.publish( _marker_msg );

    ROS_INFO_THROTTLE(5, "time for cloud_cb(%g): %g ms",
                      _cluster_tolerance, timer.getTimeMilliseconds());
  } // end cloud_cb()


  ros::NodeHandle _nh_public, _nh_private;
  ros::Subscriber _cloud_sub;
  ros::Publisher _marker_pub, _cluster_centers_pub;
  double _cluster_tolerance;
  std::vector<unsigned int> _cluster_indices;
  unsigned int _nclusters;
  std::vector<Pt2> _cluster_centers;
  geometry_msgs::PoseArray _cluster_centers_msg;
  visualization_msgs::Marker _marker_msg;
}; // end class ROSClusterer

int main(int argc, char **argv) {
  ros::init(argc, argv, "2dclusterer");
  ROSClusterer clusterer;
  ros::spin();
  return 0;
}
