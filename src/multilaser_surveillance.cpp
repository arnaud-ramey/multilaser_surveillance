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
#include <visualization_msgs/Marker.h>

typedef geometry_msgs::Point Pt2;

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

class ROSMultiLaserSurveillance : public MultiLaserSurveillance<Pt2> {
public:
  ROSMultiLaserSurveillance() : _nh_private("~") {
    // retrieve scan_topics & frames
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

    // retrieve TF between each laser frame and "map" frame
    tf::TransformListener listener;
    for (unsigned int i = 0; i < ndev; ++i) {
      tf::StampedTransform transform;
      try{
        listener.waitForTransform("/map", frames[i], ros::Time(0), ros::Duration(5));
        listener.lookupTransform("/map", frames[i], ros::Time(0), transform);
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
    _vis_pub = _nh_private.advertise<visualization_msgs::Marker>( "markers", 1 );
    _marker.header.frame_id = "map";
    _marker.id = 0;
  }

  void scan_cb(const sensor_msgs::LaserScan::ConstPtr& scan_msg,
               unsigned int device_idx) {
    // DEBUG_PRINT("scan_cb(%i)\n", device_idx);
    convert_sensor_data_to_xy(*scan_msg, _buffer);
    update_scan(device_idx, _buffer);
    publish_devices_as_markers();
  }

  void publish_devices_as_markers() {
    if (_last_vis_pub.getTimeSeconds() < 1) // 1 Hz
      return;
    DEBUG_PRINT("publish_devices_as_markers(%g)\n",
                _last_vis_pub.getTimeSeconds());
    _last_vis_pub.reset();
    _marker.ns = "devices";
    _marker.header.stamp = ros::Time::now();
    _marker.type = visualization_msgs::Marker::ARROW;
    _marker.action = visualization_msgs::Marker::ADD;
    _marker.pose.position.z = 0;
    _marker.scale.x = 1;
    _marker.scale.y = 0.1;
    _marker.scale.z = 0.1;
    _marker.color.a = 1.0; // Don't forget to set the alpha!
    _marker.color.r = 1; // yellow
    _marker.color.g = 1;
    _marker.color.b = 0.0;
    for (unsigned int i = 0; i < ndevices(); ++i) {
      Device* d = &(_devices[i]);
      _marker.id = i; // unique identifier
      _marker.pose.position = d->pos;
      _marker.pose.orientation = tf::createQuaternionMsgFromYaw(d->orien);
      _vis_pub.publish( _marker );
    } // end for i
  }

  Scan _buffer;
  ros::NodeHandle _nh_public, _nh_private;
  std::vector<ros::Subscriber> _scan_subs;
  ros::Publisher _vis_pub;
  Timer _last_vis_pub;
  visualization_msgs::Marker _marker;
}; // end class ROSMultiLaserSurveillance

////////////////////////////////////////////////////////////////////////////////

int main (int argc, char** argv) {
  ros::init(argc, argv, "multilaser_surveillance"); //Initialise and create a ROS node
  ROSMultiLaserSurveillance MultiLaserSurveillance;
  ros::spin();
}
