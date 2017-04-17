/*!
  \file         map_builder_watcher.cpp
  \author       Arnaud Ramey <arnaud.a.ramey@gmail.com>
                -- Robotics Lab, University Carlos III of Madrid
  \date         2016/09/05

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
#include <multilaser_surveillance/map_builder_watcher.h>
#include <vision_utils/convert_sensor_data_to_xy.h>
#include <vision_utils/to_lowercase.h>
#include <vision_utils/timer.h>
// ROS
#include <tf/transform_listener.h>
// ROS msg
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/OccupancyGrid.h>

typedef geometry_msgs::Point32 Pt2;

static inline void createColorMsg
(std_msgs::ColorRGBA & color, const float & red, const float & green,
 const float & blue, const float & alpha = 1) {
  color.r = red;
  color.g = green;
  color.b = blue;
  color.a = alpha;
}

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

class ROSMapBuilderWatcher : public MapBuilderWatcher<Pt2> {
public:
  ROSMapBuilderWatcher() : _nh_private("~") {
    _static_frame = "/map";
    _nh_private.param("static_frame", _static_frame, _static_frame);
    // retrieve scan_topics
    std::string scan_topics_str = "", frames_str = "";
    _nh_private.param("scan_topics", scan_topics_str, scan_topics_str);
    std::vector<std::string> scan_topics, frames;
    StringSplit(scan_topics_str, ";", &scan_topics);
    // retrieve frames
    _nh_private.param("frames", frames_str, frames_str);
    StringSplit(frames_str, ";", &frames);
    // check params validity
    if (scan_topics.empty() || frames.empty()) {
      ROS_FATAL("You must supply both 'scan_topics' and 'frames' params. Cannot init!");
      ros::shutdown();
    }
    unsigned int ndev = scan_topics.size();
    if (frames.size() != ndev) {
      ROS_FATAL("Supplied %i scan topics and %i frames, cannot init!",
                ndev, (int) frames.size());
      ros::shutdown();
    }

    // get map prefix
    _map_prefix = "mymap";
    _nh_private.param("map_prefix", _map_prefix, _map_prefix);
    // get mode
    std::string mode_str = "surveillance";
    _nh_private.param("mode", mode_str, mode_str);
    vision_utils::to_lowercase(mode_str);
    DEBUG_PRINT("mode_str:'%s'\n", mode_str.c_str());
    _mode = MODE_SURVEILLANCE;
    if (mode_str.find("auto") != std::string::npos)
      _mode = MODE_AUTO_BUILD_MAP;
    else if (mode_str.find("build") != std::string::npos)
      _mode = MODE_BUILD_MAP;
    else if (mode_str.find("surveillance") != std::string::npos)
      _mode = MODE_SURVEILLANCE;
    else {
      ROS_FATAL("Unknown mode '%s'", mode_str.c_str());
      ros::shutdown();
    }

    // get map params
    if (_mode == MODE_BUILD_MAP || _mode == MODE_AUTO_BUILD_MAP) {
      double xmin, ymin, xmax, ymax, pix2m, inflation_radius;
      _nh_private.param("auto_mode_timeout", _auto_mode_timeout, 10.); // seconds
      _nh_private.param("xmin", xmin, -10.);
      _nh_private.param("xmax", xmax, 10.);
      _nh_private.param("ymin", ymin, -10.);
      _nh_private.param("ymax", ymax, 10.);
      _nh_private.param("pix2m", pix2m, DEFAULT_PIX2M);
      _nh_private.param("inflation_radius", inflation_radius, DEFAULT_INFLATION_RADIUS);
      ROS_INFO("%s, static_frame:'%s', map_prefix:'%s', "
               "map window:(%g, %g)-(%g, %g), pix2m:%g m/pix, inflation_radius:%g m, scan_topics:%s'",
               (_mode == MODE_BUILD_MAP ? "MODE_BUILD_MAP" : "MODE_AUTO_BUILD_MAP"),
               _static_frame.c_str(), _map_prefix.c_str(),
               xmin, ymin, xmax, ymax, pix2m, inflation_radius, scan_topics_str.c_str());
      // init map
      _obstacle_map.create(xmin, ymin, xmax, ymax, pix2m, inflation_radius);
    }
    else {
      ROS_INFO("MODE_SURVEILLANCE, static_frame:'%s', map_prefix:'%s', scan_topics:%s'",
               _static_frame.c_str(), _map_prefix.c_str(), scan_topics_str.c_str());
      if (!_obstacle_map.load(_map_prefix)) {
        ROS_FATAL("Could not load map!");
        ros::shutdown();
      }
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
      add_device(MapBuilderWatcher::SurveillanceDevice(scan_topics[i], p, tf::getYaw(q)));
    } // end for i

    // create subscribers
    _scan_subs.resize(ndev);
    for (unsigned int i = 0; i < ndev; ++i) {
      // create subscribers - pass i
      // http://ros-users.122217.n3.nabble.com/How-to-identify-the-subscriber-or-the-topic-name-related-to-a-callback-td2391327.html
      _scan_subs[i] = _nh_public.subscribe<sensor_msgs::LaserScan>
          (scan_topics[i], 0,
           boost::bind(&ROSMapBuilderWatcher::scan_cb, this, _1, i));
    }

    // create publishers
    _map_pub = _nh_public.advertise<nav_msgs::OccupancyGrid>( "map", 0 );
    _marker_pub = _nh_public.advertise<visualization_msgs::Marker>( "marker", 0 );
    _outliers_pub = _nh_public.advertise<sensor_msgs::PointCloud>( "outliers", 0 );
    _scan_pub = _nh_public.advertise<sensor_msgs::PointCloud>( "scan", 0 );
    _marker_msg.header.frame_id = _static_frame;
    _marker_msg.id = 0;
    _map_msg.header = _marker_msg.header;
    _outliers_msg.header = _marker_msg.header;
    _scan_msg.header = _marker_msg.header;
  }

protected:
  static inline geometry_msgs::Point Pt2ToPoint(const Pt2 & pt) {
    geometry_msgs::Point out;
    out.x = pt.x;
    out.y = pt.y;
    return out;
  }

  //////////////////////////////////////////////////////////////////////////////

  void scan_cb(const sensor_msgs::LaserScan::ConstPtr& scan_msg,
               unsigned int device_idx) {
    // DEBUG_PRINT("scan_cb(%i)\n", device_idx);
    vision_utils::Timer timer;
    Scan _buffer;
    vision_utils::convert_sensor_data_to_xy(*scan_msg, _buffer);
    update_scan(device_idx, _buffer);
    publish_outliers();
    publish_scan();
    publish_and_save_map();
    publish_devices_as_markers();
    ROS_INFO_THROTTLE(5, "time for scan_cb(): %g ms", timer.getTimeMilliseconds());
  }

  //////////////////////////////////////////////////////////////////////////////

  void publish_outliers() {
    if (_mode == MODE_BUILD_MAP
        || _mode == MODE_AUTO_BUILD_MAP
        || _outliers_pub.getNumSubscribers() == 0
        || _outliers_timer.getTimeSeconds() < .01) // 100 Hz
      return;
    _outliers_timer.reset();
    _outliers_msg.header.stamp = ros::Time::now();
    recompute_outliers_if_needed();
    _outliers_msg.points = _outliers;
    // http://docs.ros.org/kinetic/api/sensor_msgs/html/msg/ChannelFloat32.html
    _outliers_msg.channels.resize(1);
    _outliers_msg.channels.front().name = "laser_ids";
    _outliers_msg.channels.front().values = _outlier_laser_ids;
    _outliers_pub.publish(_outliers_msg);
  }

  //////////////////////////////////////////////////////////////////////////////

  void publish_scan() {
    // DEBUG_PRINT("publish_scan()\n");
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
    if (_marker_pub.getNumSubscribers() == 0
        || _marker_timer.getTimeSeconds() < 1) // 1 Hz
      return;
    //DEBUG_PRINT("publish_devices_as_markers()\n");
    _marker_timer.reset();
    _marker_msg.ns = "devices";
    _marker_msg.header.stamp = ros::Time::now();
    _marker_msg.type = visualization_msgs::Marker::ARROW;
    _marker_msg.action = visualization_msgs::Marker::ADD;
    _marker_msg.pose.position.z = 0;
    _marker_msg.scale.x = 1;
    _marker_msg.scale.y = 0.2;
    _marker_msg.scale.z = 0.2;
    for (unsigned int i = 0; i < ndevices(); ++i) {
      SurveillanceDevice* d = &(_devices[i]);
      if (d->_last_scan.empty()) // never got a scal
        createColorMsg(_marker_msg.color, 1, 0, 0); // red
      else if (d->_last_scan_timer.getTimeSeconds() > 1)
        createColorMsg(_marker_msg.color, 1, 1, 0); // yellow
      else // all ok
        createColorMsg(_marker_msg.color, 0, 1, 0); // green
      _marker_msg.id = i; // unique identifier
      _marker_msg.pose.position = Pt2ToPoint(d->_pos);
      _marker_msg.pose.orientation = tf::createQuaternionMsgFromYaw(d->_orien);
      _marker_pub.publish( _marker_msg );
    } // end for i
  }

  //////////////////////////////////////////////////////////////////////////////

  void publish_and_save_map() {
    if (_map_pub.getNumSubscribers() == 0
        || _map_timer.getTimeSeconds() < 1) // 1 Hz
      return;
    _map_timer.reset();
    // DEBUG_PRINT("publish_map()\n");
    // recreate map if needed
    unsigned int w = _obstacle_map.get_width(), h = _obstacle_map.get_height();
    if (_mode == MODE_BUILD_MAP
        || _mode == MODE_AUTO_BUILD_MAP
        || _map_msg.info.height != h
        || _map_msg.info.width != w) {
      _map_msg.info.map_load_time = ros::Time::now();
      _map_msg.info.origin.position.x = _obstacle_map.get_xmin();
      _map_msg.info.origin.position.y = _obstacle_map.get_ymin();
      _map_msg.info.origin.orientation = tf::createQuaternionMsgFromYaw(0);
      _map_msg.info.resolution = _obstacle_map.get_pix2m();
      _map_msg.info.width = w;
      _map_msg.info.height = h;
      _obstacle_map.export2vector(_map_msg.data);
    }
    _map_msg.header.stamp = ros::Time::now();
    _map_pub.publish( _map_msg );
    // save map
    if ((_mode == MODE_BUILD_MAP || _mode == MODE_AUTO_BUILD_MAP)
        && !_obstacle_map.save(_map_prefix)) {
      ROS_WARN("Could not save map into '%s'", _map_prefix.c_str());
    }
  }

  //////////////////////////////////////////////////////////////////////////////
  //////////////////////////////////////////////////////////////////////////////

  std::string _static_frame, _map_prefix;
  ros::NodeHandle _nh_public, _nh_private;
  std::vector<ros::Subscriber> _scan_subs;
  ros::Publisher _marker_pub, _scan_pub, _outliers_pub, _map_pub;
  vision_utils::Timer _marker_timer, _scan_timer, _outliers_timer, _map_timer;
  visualization_msgs::Marker _marker_msg;
  sensor_msgs::PointCloud _scan_msg, _outliers_msg;
  nav_msgs::OccupancyGrid _map_msg;
}; // end class ROSMapBuilderWatcher

////////////////////////////////////////////////////////////////////////////////

int main (int argc, char** argv) {
  ros::init(argc, argv, "map_builder_watcher"); //Initialise and create a ROS node
  ROSMapBuilderWatcher mbw;
  ros::spin();
}
