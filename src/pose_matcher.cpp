/*!
  \file         pose_matcher.cpp
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
// ROS
#include <multilaser_surveillance/map_builder_watcher.h>
#include <ros/ros.h>
// ROS msg
#include <geometry_msgs/PoseArray.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float32.h>

std::vector<nav_msgs::Odometry> _last_ground_truths;
std::vector<ros::Subscriber> _odom_subs;
ros::Publisher dist_pub;

void ground_truth_cb(const nav_msgs::Odometry::ConstPtr& odom_msg,
                     unsigned int robot_idx) {
  _last_ground_truths[robot_idx] = *odom_msg;
}

//////////////////////////////////////////////////////////////////////////////

void estimated_poses_cb(const geometry_msgs::PoseArray::ConstPtr& poses_msg) {
  //ROS_INFO("estimated_poses_cb()");
  unsigned int nposes = poses_msg->poses.size(), ntruths = _last_ground_truths.size();
  std_msgs::Float32 dist_msg;
  dist_msg.data = 0;
  ros::Time now  = ros::Time::now();
  for (unsigned int pi = 0; pi < nposes; ++pi) {
    double mindist = 1E6;
    geometry_msgs::Point estimated_pt = poses_msg->poses[pi].position;
    for (unsigned int oi = 0; oi < ntruths; ++oi) {
      if ((now - _last_ground_truths[oi].header.stamp).toSec() > 1)
        continue;
      geometry_msgs::Point truth_pt = _last_ground_truths[oi].pose.pose.position;
      double currdist = hypot(truth_pt.x - estimated_pt.x, 0);
      if (mindist > currdist)
        mindist = currdist;
    } // end for oi
    dist_msg.data += mindist;
  } // end for pi
  dist_msg.data  *= 1. / nposes; // normalize
  dist_pub.publish(dist_msg);
}

//////////////////////////////////////////////////////////////////////////////

int main(int argc, char **argv) {
  ros::init(argc, argv, "pose_matcher");
  ros::NodeHandle nh_public, nh_private("~");
  // retrieve ground_truth_topics
  std::string ground_truth_topics_str = "";
  nh_private.param("ground_truth_topics", ground_truth_topics_str, ground_truth_topics_str);
  std::vector<std::string> ground_truth_topics;
  StringSplit(ground_truth_topics_str, ";", &ground_truth_topics);
  unsigned int nrobots = ground_truth_topics.size();

  // create publishers
  dist_pub = nh_private.advertise<std_msgs::Float32>("dist", 1);
  // create subscribers
  _odom_subs.resize(nrobots);
  _last_ground_truths.resize(nrobots);
  for (unsigned int i = 0; i < nrobots; ++i) {
    // create subscribers - pass i
    // http://ros-users.122217.n3.nabble.com/How-to-identify-the-subscriber-or-the-topic-name-related-to-a-callback-td2391327.html
    _odom_subs[i] = nh_public.subscribe<nav_msgs::Odometry>
        (ground_truth_topics[i], 0, boost::bind(&ground_truth_cb, _1, i));
  }
  ros::Subscriber estimated_poses_sub = nh_private.subscribe<geometry_msgs::PoseArray>
      ("estimated_poses", 1, estimated_poses_cb);

  ROS_INFO("pose_matcher: '%s' <=> '%s'", ground_truth_topics_str.c_str(),
           estimated_poses_sub.getTopic().c_str());
  ros::spin();
  return 0;
}
