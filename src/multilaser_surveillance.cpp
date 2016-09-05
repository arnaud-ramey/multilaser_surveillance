/*!
  \file         multilaser_surveillance.cpp
  \author       Arnaud Ramey <arnaud.a.ramey@gmail.com>
                -- Robotics Lab, University Carlos III of Madrid
  \date         2015/10/14

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


\section Parameters
*/
#include <multilaser_surveillance/wanderer.h>
// ROS
#include <tf/transform_listener.h>
// ROS msg
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>
#include <visualization_msgs/Marker.h>

typedef geometry_msgs::Point32 Pt2;

////////////////////////////////////////////////////////////////////////////////

//! convert from polar to xy coordinates for a laser data
template<class _Pt2>
static inline void convert_sensor_data_to_xy(const sensor_msgs::LaserScan & laser_msg,
                                             std::vector<_Pt2> & out_vector) {
  out_vector.clear();
  out_vector.reserve(laser_msg.ranges.size());
  const float* curr_range = &(laser_msg.ranges[0]);
  float curr_angle = laser_msg.angle_min;
  for (unsigned int idx = 0; idx < laser_msg.ranges.size(); ++idx) {
    //maggieDebug2("idx:%i, curr_range:%g", idx, *curr_range);
    _Pt2 pt;
    pt.x = *curr_range * cos(curr_angle);
    pt.y = *curr_range * sin(curr_angle);
    out_vector.push_back(pt);
    ++curr_range;
    curr_angle += laser_msg.angle_increment;
  } // end loop idx
} // end convert_sensor_data_to_xy()

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

class ROSMultiLaserSurveillance : public MultiLaserSurveillance<Pt2> {
public:
  ROSMultiLaserSurveillance() : _nh_private("~") {
    // retrieve scan_topics & frames
    _static_frame = "/map";
    std::string scan_topics_str = "", frames_str = "";
    _nh_private.param("scan_topics", scan_topics_str, scan_topics_str);
    _nh_private.param("frames", frames_str, frames_str);
    std::vector<std::string> scan_topics, frames;
    StringSplit(scan_topics_str, ";", &scan_topics);
    StringSplit(frames_str, ";", &frames);

    // check params validity
    unsigned int ndev = scan_topics.size();
    if (frames.size() != ndev) {
      ROS_FATAL("Supplied %i scan topics and %i frames, cannot init!",
                ndev, (int) frames.size());
      ros::shutdown();
    }
    if (frames.empty()) {
      ROS_FATAL("You must supply both 'scan_topics' and 'frames' params. Cannot init!");
      ros::shutdown();
    }

    // retrieve TF between each laser frame and _static_frame
    tf::TransformListener listener;
    for (unsigned int i = 0; i < ndev; ++i) {
      tf::StampedTransform transform;
      try{
        listener.waitForTransform(_static_frame, frames[i], ros::Time(0), ros::Duration(5));
        listener.lookupTransform(_static_frame, frames[i], ros::Time(0), transform);
      }
      catch (tf::TransformException ex){
        ROS_FATAL("Cannot get TF, %s",ex.what());
        ros::shutdown();
      }
      tf::Vector3 pv = transform.getOrigin();
      Pt2 p; p.x = pv[0]; p.y = pv[1];
      tf::Quaternion q = transform.getRotation();
      add_device(p, tf::getYaw(q));
    } // end for i

    // create subscribers
    _scan_subs.resize(ndev);
    for (unsigned int i = 0; i < ndev; ++i) {
      // create subscribers - pass i
      // http://ros-users.122217.n3.nabble.com/How-to-identify-the-subscriber-or-the-topic-name-related-to-a-callback-td2391327.html
      _scan_subs[i] = _nh_public.subscribe<sensor_msgs::LaserScan>
          (scan_topics[i], 1,
           boost::bind(&ROSMultiLaserSurveillance::scan_cb, this, _1, i));
    }

    // create publishers
    _markers_pub = _nh_private.advertise<visualization_msgs::Marker>( "markers", 1 );
    _scan_pub = _nh_private.advertise<sensor_msgs::PointCloud>( "scan", 1 );
    _outliers_pub = _nh_private.advertise<sensor_msgs::PointCloud>( "outliers", 1 );
    _marker_msg.header.frame_id = _static_frame;
    _scan_msg.header = _marker_msg.header;
    _outliers_msg.header = _marker_msg.header;
    _marker_msg.id = 0;
  }

protected:
  inline geometry_msgs::Point Pt2ToPoint(const Pt2 & pt) {
    geometry_msgs::Point out;
    out.x = pt.x; 
    out.y = pt.y; 
    return out;
  }

  //////////////////////////////////////////////////////////////////////////////

  void scan_cb(const sensor_msgs::LaserScan::ConstPtr& scan_msg,
               unsigned int device_idx) {
    // DEBUG_PRINT("scan_cb(%i)\n", device_idx);
    convert_sensor_data_to_xy(*scan_msg, _buffer);
    update_scan(device_idx, _buffer);
    publish_outliers();
    publish_scan();
    publish_devices_as_markers();
  }

  //////////////////////////////////////////////////////////////////////////////

  void publish_outliers() {
    if (_outliers_pub.getNumSubscribers() == 0
        || _outliers_timer.getTimeSeconds() < .01) // 100 Hz
      return;
    _outliers_timer.reset();
    _outliers_msg.header.stamp = ros::Time::now();
    recompute_outliers_if_needed();
    _outliers_msg.points = _outliers;
    _outliers_pub.publish(_outliers_msg);
  }

  //////////////////////////////////////////////////////////////////////////////

  void publish_scan() {
    if (_scan_pub.getNumSubscribers() == 0
        || _scan_timer.getTimeSeconds() < .01) // 100 Hz
      return;
    _scan_timer.reset();
    _scan_msg.header.stamp = ros::Time::now();
    recompute_scan_if_needed();
    _scan_msg.points = _scan;
    _scan_pub.publish(_scan_msg);
  }

  //////////////////////////////////////////////////////////////////////////////

  void publish_devices_as_markers() {
    if (_markers_pub.getNumSubscribers() == 0
        || _markers_timer.getTimeSeconds() < 1) // 1 Hz
      return;
    DEBUG_PRINT("publish_devices_as_markers(%g)\n",
                _markers_timer.getTimeSeconds());
    _markers_timer.reset();
    _marker_msg.ns = "devices";
    _marker_msg.header.stamp = ros::Time::now();
    _marker_msg.type = visualization_msgs::Marker::ARROW;
    _marker_msg.action = visualization_msgs::Marker::ADD;
    _marker_msg.pose.position.z = 0;
    _marker_msg.scale.x = 1;
    _marker_msg.scale.y = 0.1;
    _marker_msg.scale.z = 0.1;
    _marker_msg.color.a = 1.0; // Don't forget to set the alpha!
    _marker_msg.color.r = 1; // yellow
    _marker_msg.color.g = 1;
    _marker_msg.color.b = 0.0;
    for (unsigned int i = 0; i < ndevices(); ++i) {
      Device* d = &(_devices[i]);
      _marker_msg.id = i; // unique identifier
      _marker_msg.pose.position = Pt2ToPoint(d->pos);
      _marker_msg.pose.orientation = tf::createQuaternionMsgFromYaw(d->orien);
      _markers_pub.publish( _marker_msg );
    } // end for i
  }

  //////////////////////////////////////////////////////////////////////////////
  //////////////////////////////////////////////////////////////////////////////

  std::string _static_frame;
  Scan _buffer;
  ros::NodeHandle _nh_public, _nh_private;
  std::vector<ros::Subscriber> _scan_subs;
  ros::Publisher _markers_pub, _scan_pub, _outliers_pub;
  Timer _markers_timer, _scan_timer, _outliers_timer;
  visualization_msgs::Marker _marker_msg;
  sensor_msgs::PointCloud _scan_msg, _outliers_msg;
}; // end class ROSMultiLaserSurveillance

////////////////////////////////////////////////////////////////////////////////

int main (int argc, char** argv) {
  ros::init(argc, argv, "multilaser_surveillance"); //Initialise and create a ROS node
  ROSMultiLaserSurveillance MultiLaserSurveillance;
  ros::spin();
}
