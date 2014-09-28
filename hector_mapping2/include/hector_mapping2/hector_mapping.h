//=================================================================================================
// Copyright (c) 2013, Johannes Meyer and contributors, Technische Universitat Darmstadt
// All rights reserved.

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of the Flight Systems and Automatic Control group,
//       TU Darmstadt, nor the names of its contributors may be used to
//       endorse or promote products derived from this software without
//       specific prior written permission.

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//=================================================================================================


#ifndef HECTOR_MAPPING2_NODE_H
#define HECTOR_MAPPING2_NODE_H

#include <hector_mapping2_core/map/types.h>
#include <hector_mapping2_core/scan.h>
#include <hector_mapping2_core/matcher.h>

#include <ros/ros.h>
#include <nodelet/nodelet.h>

#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <std_msgs/String.h>

#include <nav_msgs/GetMap.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

namespace hector_mapping {

class Node : public nodelet::Nodelet
{
public:
  Node();
  virtual ~Node();

  void onInit();
  void reset();

  void scanCallback(const sensor_msgs::LaserScanConstPtr& scan);
  void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud);
  void initialPoseCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr& initial_pose);
  void staticMapCallback(const nav_msgs::OccupancyGridConstPtr& static_map);
  void syscommandCallback(const std_msgs::StringConstPtr& syscommand);
  bool mapServiceCallback(nav_msgs::GetMap::Request& request, nav_msgs::GetMap::Response& response);

  void publishMap();
  void publishPose();
  void publishTf();

  void mapPublishThread(ros::Rate rate);

protected:
  bool update();

protected:
  Parameters parameters_;
  std::string p_map_frame_;
  std::string p_base_frame_;
  std::string p_odom_frame_;
  bool p_use_tf_scan_transformation_;
  bool p_use_tf_pose_start_estimate_;
  bool p_pub_map_odom_transform_;
  bool p_advertise_map_service_;
  double p_map_update_translational_threshold_;
  double p_map_update_angular_threshold_;

  OccupancyGridMapPtr map_;
  ScanMatcherPtr matcher_;
  Scan scan_;

  tf::Transform last_map_update_pose_;
  tf::StampedTransform map_odom_transform_;
  ros::Time last_map_update_time_;

  boost::shared_ptr<tf::TransformBroadcaster> tf_broadcaster_;
  tf::TransformBroadcaster &getTransformBroadcaster();
  boost::shared_ptr<tf::TransformListener> tf_listener_;
  tf::TransformListener &getTransformListener();

  nav_msgs::OccupancyGrid map_message_;

private:
  ros::Subscriber scan_subscriber_;
  ros::Subscriber cloud_subscriber_;
  ros::Subscriber initial_pose_subscriber_;
  ros::Subscriber static_map_subscriber_;
  ros::Subscriber syscommand_subscriber_;
  ros::Publisher pose_with_covariance_publisher_;
  ros::Publisher pose_publisher_;
  ros::Publisher covariance_publisher_;

  ros::Publisher map_publisher_;
  ros::Publisher map_metadata_publisher_;
  boost::thread map_publish_thread_;

  ros::ServiceServer map_service_;

  ros::Publisher timing_publisher_;
};

} // namespace hector_mapping

#endif // HECTOR_MAPPING2_NODE_H
