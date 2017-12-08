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
#include <multilaser_surveillance/string_split.h>
#include <vision_utils/timer.h>
#include <vision_utils/copy2.h>
// ROS
#include <tf/transform_listener.h>
// ROS msg
#include <std_msgs/Header.h>
#include <std_msgs/String.h>
#include <geometry_msgs/PoseArray.h>
#include <people_msgs/People.h>
#include <sensor_msgs/PointCloud.h>
#include <visualization_msgs/Marker.h>

typedef geometry_msgs::Point32 Pt2;

/*!
 * \brief   detect if a point is inside a polygon - return true or false
 *  http://en.wikipedia.org/wiki/Point_in_polygon#Winding_number_algorithm
 * algo from http://www.visibone.com/inpoly/
     *
 * \param   p the point
 * \param   poly the polygon
 * \return  true if the point is in the polygon
 */
template<class Point2>
static inline bool point_inside_polygon(const Point2 & p,
                                        const std::vector<Point2> & poly) {
  Point2 p_old, p_new, p1, p2;
  bool inside = false;
  int npoints = poly.size();

  if (npoints < 3) {
    return(0);
  }
  p_old = poly[npoints-1];

  for (int i=0 ; i < npoints ; i++) {
    p_new = poly[i];
    if (p_new.x > p_old.x) {
      p1 = p_old;
      p2 = p_new;
    }
    else {
      p1 = p_new;
      p2 = p_old;
    }
    if ((p_new.x < p.x) == (p.x <= p_old.x)          /* edge "open" at one end */
        && 1.f * (p.y-p1.y) * (p2.x-p1.x) < 1.f * (p2.y-p1.y) * (p.x-p1.x)) {
      inside = !inside;
    }
    p_old.x = p_new.x;
    p_old.y = p_new.y;
  } // end loop i
  return(inside);
} // end point_inside_polygon

////////////////////////////////////////////////////////////////////////////////

template<class Pt2a, class Pt2b>
void copy_vec2(const std::vector<Pt2a> & src, std::vector<Pt2b> & dst) {
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
  static const unsigned int NPTS_PER_CIRCLE_MARKER = 20;
  enum CenterComputation {
    CENTER_COMPUTATION_METHOD_BARYCENTER = 0,
    CENTER_COMPUTATION_METHOD_FIT_CIRCLE = 1
  };

  //////////////////////////////////////////////////////////////////////////////

  ROSClusterer() : _nh_private("~") {
    _nh_private.param("cluster_tolerance", _cluster_tolerance, .1);
    int min_pts_per_cluster_int; // param() doesn't take "unsigned ints" as inputs
    _nh_private.param("min_pts_per_cluster", min_pts_per_cluster_int, 1);
    _min_pts_per_cluster = min_pts_per_cluster_int;
    //ROS_WARN("min_pts_per_cluster:%i", _min_pts_per_cluster);
    int max_clusters_int; // param() doesn't take "unsigned ints" as inputs
    _max_clusters = max_clusters_int;
    _nh_private.param("max_clusters", max_clusters_int, 20);
    _nh_private.param("objects_radius", _objects_radius, .5);
    std::string center_computation_method_str = "fit";
    _nh_private.param("center_computation_method", center_computation_method_str, center_computation_method_str);
    _center_computation_method = CENTER_COMPUTATION_METHOD_FIT_CIRCLE;
    if (center_computation_method_str.find("barycenter") != std::string::npos)
      _center_computation_method = CENTER_COMPUTATION_METHOD_BARYCENTER;
    // retrieve green zone
    std::string green_zone_str = "";
    _nh_private.param("green_zone", green_zone_str, green_zone_str);
    std::vector<std::string> green_zone_coords;
    std::ostringstream green_info;
    StringSplit(green_zone_str, ";", &green_zone_coords);
    if (green_zone_coords.size() % 2 == 1) {
      ROS_WARN("You should specify pairs of coordinates, param '%s' is invalid", green_zone_str.c_str());
    }
    else for (unsigned int i = 0; i < green_zone_coords.size()/2; ++i) {
      Pt2 green_corner;
      green_corner.x = atof(green_zone_coords[2*i  ].c_str());
      green_corner.y = atof(green_zone_coords[2*i+1].c_str());
      _green_zone.push_back(green_corner);
      green_info << "(" << green_corner.x << ", " << green_corner.y << "), ";
    }
    ROS_INFO("Green zone of %li corners:%s", _green_zone.size(), green_info.str().c_str());

    _cloud_sub = _nh_public.subscribe<sensor_msgs::PointCloud>
        ("cloud", 0, &ROSClusterer::cloud_cb, this);
    _cluster_centers_pub = _nh_private.advertise<geometry_msgs::PoseArray>( "cluster_centers", 0 );
    _nclusters_pub = _nh_private.advertise<std_msgs::Header>( "nclusters", 0 );
    _tops_pub = _nh_private.advertise<std_msgs::Header>( "/tops", 0 );
    _tts_pub = _nh_private.advertise<std_msgs::String>( "/tts", 0 );
    _ppl_pub = _nh_private.advertise<people_msgs::People>( "cluster_ppl", 0 );
    _marker_pub = _nh_private.advertise<visualization_msgs::Marker>( "marker", 0 );
    _radiuscos.resize(NPTS_PER_CIRCLE_MARKER);
    _radiussin.resize(NPTS_PER_CIRCLE_MARKER);
    for (unsigned int j = 0; j < NPTS_PER_CIRCLE_MARKER; ++j) {
      double theta = j * 2 * M_PI / (NPTS_PER_CIRCLE_MARKER-1);
      _radiuscos[j] = _objects_radius * cos(theta);
      _radiussin[j] = _objects_radius * sin(theta);
    } // end for j
    _marker_msg.lifetime = ros::Duration(1);

    ROS_INFO("ROSClusterer: cluster_tolerance:%g m, min %i pts per cluster,"
             "max: %i clusters, center_computation_method:%i, objects_radius:%g m",
             _cluster_tolerance, _min_pts_per_cluster, _max_clusters,
             _center_computation_method, _objects_radius);
  }

  //////////////////////////////////////////////////////////////////////////////

  void cloud_cb(const sensor_msgs::PointCloud::ConstPtr& cloud_msg) {
    //ROS_WARN("cloud_cb() - %i subscribers", _cluster_centers_pub.getNumSubscribers());
    vision_utils::Timer timer;
    if (!cluster(cloud_msg->points, _cluster_indices,
                 _nclusters, _cluster_tolerance)) {
      printf( "cluster() returned an error.\n");
      return;
    }
    if (_center_computation_method == CENTER_COMPUTATION_METHOD_BARYCENTER
        && !barycenters(cloud_msg->points, _cluster_indices, _nclusters,
                        _min_pts_per_cluster, _cluster_centers)) {
      printf( "barycenters() returned an error.\n");
      return;
    }
    else if (_center_computation_method == CENTER_COMPUTATION_METHOD_FIT_CIRCLE
             && !best_fit_circles(cloud_msg->points, _cluster_indices,
                                  _nclusters, _min_pts_per_cluster,
                                  _objects_radius, _cluster_centers)) {
      printf( "best_fit_circles() returned an error.\n");
      return;
    }
    // check green zone if needed
    if (_green_zone.size() >= 3) {
      for (unsigned int i = 0; i < _nclusters; ++i) {
        if (point_inside_polygon(_cluster_centers[i], _green_zone))
          continue;
        _cluster_centers.erase(_cluster_centers.begin()+i);
        --_nclusters;
        --i; // rewind
      }
    }

    if (_nclusters > _max_clusters) {
      ROS_WARN_THROTTLE(1, "Got %i clusters, while threshold is %i. "
                        "Not publishing found clusters", _nclusters, _max_clusters);
      return;
    }
    //printf("nclusters:%i\n", _nclusters);
    // publish _ppl
    if (_ppl_pub.getNumSubscribers()) {
      _ppl.header = cloud_msg->header;
      _ppl.people.resize(_nclusters);
      for (unsigned int ci = 0; ci < _nclusters; ++ci) {
        vision_utils::copy2(_cluster_centers[ci], _ppl.people[ci].position);
        _ppl.people[ci].reliability = 1;
        _ppl.people[ci].tagnames.resize(1, "method");
        _ppl.people[ci].tags.resize(1, "2dclusterer");
      }
      _ppl_pub.publish(_ppl);
    } // end if (_cluster_centers_pub.getNumSubscribers())

    // publish _nclusters_msg
    if (_nclusters_pub.getNumSubscribers()) {
      _nclusters_msg = cloud_msg->header;
      _nclusters_msg.seq = _nclusters;
      _nclusters_pub.publish(_nclusters_msg);
    } // end if (_nclusters_pub.getNumSubscribers())

    // publish tops and tts
    if (_nclusters != _tops_msg.seq) {
      unsigned int prev_n = _tops_msg.seq;
      _tops_msg = cloud_msg->header;
      _tops_msg.seq = _nclusters;
      _tops_pub.publish(_tops_msg);
      std_msgs::String tts_msg;
      std::ostringstream msg;
      msg << "Top " << (_nclusters > prev_n ? "entrée " : "sortie ")
          << _nh_public.getNamespace().substr(2);
      tts_msg.data = msg.str();
      _tts_pub.publish(tts_msg);
    } // end if (_tops_pub.getNumSubscribers())

    // publish _cluster_centers_msg
    if (_cluster_centers_pub.getNumSubscribers()) {
      _cluster_centers_msg.header = cloud_msg->header;
      _cluster_centers_msg.poses.resize(_nclusters);
      for (unsigned int ci = 0; ci < _nclusters; ++ci) {
        vision_utils::copy2(_cluster_centers[ci],
                            _cluster_centers_msg.poses[ci].position);
        _cluster_centers_msg.poses[ci].orientation.z = 1;
      } // end for ci
      _cluster_centers_pub.publish(_cluster_centers_msg);
    } // end if (_cluster_centers_pub.getNumSubscribers())

    // publish markers
    if (_marker_pub.getNumSubscribers()) {
      _marker_msg.header = cloud_msg->header;
      _marker_msg.colors.clear();

      // publish green zone marker
      // Line strips use the points member of the visualization_msgs/Marker message.
      // It will draw a line between every two consecutive points, so 0-1, 1-2, 2-3, 3-4, 4-5...
      if (_green_zone.size() >= 3) {
        _marker_msg.color.r = 0;
        _marker_msg.color.b = (_nclusters > 0 ? 0 : 1);
        _marker_msg.color.g = (_nclusters > 0 ? 1 : 0); // green if sbdy, blue otherwise
        _marker_msg.color.a = 1; // no transparency
        _marker_msg.ns = "green_zone";
        _marker_msg.type = visualization_msgs::Marker::LINE_STRIP;
        _marker_msg.action = visualization_msgs::Marker::ADD;
        _marker_msg.id = 0; // unique identifier
        _marker_msg.scale.x = 0.05;
        unsigned int ncorners = _green_zone.size();
        _marker_msg.points.resize(2*ncorners);
        _marker_msg.points[0].x = _marker_msg.points[2*ncorners-1].x = _green_zone[0].x;
        _marker_msg.points[0].y = _marker_msg.points[2*ncorners-1].y = _green_zone[0].y;
        _marker_msg.points[0].z = _marker_msg.points[2*ncorners-1].z = 0;
        for (unsigned int i = 1; i < ncorners; ++i) {
          _marker_msg.points[2*i-1].x = _marker_msg.points[2*i ].x = _green_zone[i].x;
          _marker_msg.points[2*i-1].y = _marker_msg.points[2*i ].y = _green_zone[i].y;
          _marker_msg.points[2*i-1].z = _marker_msg.points[2*i ].z = 0;
        }
        _marker_pub.publish( _marker_msg );
      }

      // publish a circle for each fitted circle
      _marker_msg.ns = "fitted_circles";
      _marker_msg.type = visualization_msgs::Marker::LINE_STRIP;
      _marker_msg.action = visualization_msgs::Marker::MODIFY;
      // Line strips use the points member of the visualization_msgs/Marker message.
      // It will draw a line between every two consecutive points, so 0-1, 1-2, 2-3, 3-4, 4-5...
      // Line strips also have some special handling for scale:
      // only scale.x is used and it controls the width of the line segments.
      _marker_msg.scale.x = 0.05;
      _marker_msg.points.resize(NPTS_PER_CIRCLE_MARKER);
      for (unsigned int ci = 0; ci < _nclusters; ++ci) {
        _marker_msg.id = ci; // unique identifier
        // In visualization 1.1+ will also optionally use the colors member for per-vertex color.
        indexed_color_norm(_marker_msg.color.r, _marker_msg.color.g, _marker_msg.color.b, ci);
        for (unsigned int j = 0; j < NPTS_PER_CIRCLE_MARKER; ++j) {
          // Note that pose is still used (the points in the line will be transformed by them),
          // and the lines will be correct relative to the frame id specified in the header.
          _marker_msg.points[j].x = _cluster_centers[ci].x + _radiuscos[j];
          _marker_msg.points[j].y = _cluster_centers[ci].y + _radiussin[j];
          _marker_msg.points[j].z = 0;
        } // end for j
        _marker_pub.publish( _marker_msg );
      } // end for ci
    } // end if (_marker_pub.getNumSubscribers())

    ROS_INFO_THROTTLE(5, "time for cloud_cb(%g): %g ms",
                      _cluster_tolerance, timer.getTimeMilliseconds());
  } // end cloud_cb()

  //////////////////////////////////////////////////////////////////////////////
  //////////////////////////////////////////////////////////////////////////////

  ros::NodeHandle _nh_public, _nh_private;
  ros::Subscriber _cloud_sub;
  ros::Publisher _marker_pub, _cluster_centers_pub, _nclusters_pub, _ppl_pub, _tops_pub, _tts_pub;
  double _cluster_tolerance, _objects_radius;
  std::vector<unsigned int> _cluster_indices;
  unsigned int _nclusters, _max_clusters, _min_pts_per_cluster;
  std::vector<Pt2> _green_zone;
  CenterComputation _center_computation_method;
  std::vector<Pt2> _cluster_centers;
  geometry_msgs::PoseArray _cluster_centers_msg;
  std_msgs::Header _nclusters_msg, _tops_msg;

  people_msgs::People _ppl;
  visualization_msgs::Marker _marker_msg;
  std::vector<double> _radiuscos, _radiussin;
}; // end class ROSClusterer

//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////

int main(int argc, char **argv) {
  ros::init(argc, argv, "2dclusterer");
  ROSClusterer clusterer;
  ros::spin();
  return 0;
}
