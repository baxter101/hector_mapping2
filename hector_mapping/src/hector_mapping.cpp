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

#include <hector_mapping/hector_mapping.h>
#include <hector_mapping_core/map/conversion.h>

namespace hector_mapping {

Node::Node()
  : scan_(parameters_)
{
  parameters_("map_size", Size(1024, 1024, 1024));
  parameters_("map_resolution", Resolution(0.05, 0.05, 0.05));
  parameters_("map_frame", p_map_frame_ = "map");
  parameters_("base_frame", p_base_frame_ = "base_link");
  parameters_("odom_frame", p_odom_frame_ = "base_link");
  parameters_("use_tf_scan_transformation", p_use_tf_scan_transformation_ = true);
  parameters_("pub_map_odom_transform_", p_pub_map_odom_transform_ = true);
  parameters_("advertise_map_service", p_advertise_map_service_ = true);

  parameters_("map_update_distance_thresh", p_map_update_translational_threshold_ = 0.4);
  parameters_("map_update_angle_thresh", p_map_update_angular_threshold_ = 0.9);
}

Node::~Node()
{

}

void Node::reset()
{
  last_map_update_pose_.setIdentity();
  last_map_update_time_ = ros::Time();

  if (map_) map_->reset();
  if (matcher_) matcher_->reset();

  publishMap();
  publishPose();
}

void Node::onInit()
{
  // map_size parameter
  XmlRpc::XmlRpcValue p_map_size;
  Size &map_size = parameters_.get<Size>("map_size");
  if (getPrivateNodeHandle().getParam("map_size", p_map_size)) {
    if (p_map_size.getType() == XmlRpc::XmlRpcValue::TypeInt) {
      map_size.x() = map_size.y() = static_cast<int>(p_map_size);
    } else if (p_map_size.getType() == XmlRpc::XmlRpcValue::TypeArray) {
      if (p_map_size.size() >= 2) {
        map_size.x() = static_cast<int>(p_map_size[0]);
        map_size.y() = static_cast<int>(p_map_size[1]);
      }
      if (p_map_size.size() >= 3) {
        map_size.z() = static_cast<int>(p_map_size[2]);
      }
    }
  }

  // map_resolution parameter
  XmlRpc::XmlRpcValue p_map_resolution;
  Resolution &resolution = parameters_.get<Resolution>("map_resolution");
  if (getPrivateNodeHandle().getParam("map_resolution", p_map_resolution)) {
    if (p_map_resolution.getType() == XmlRpc::XmlRpcValue::TypeDouble) {
      resolution.x() = resolution.y() = static_cast<double>(p_map_resolution);
    } else if (p_map_resolution.getType() == XmlRpc::XmlRpcValue::TypeArray) {
      if (p_map_resolution.size() >= 2) {
        resolution.x() = static_cast<double>(p_map_resolution[0]);
        resolution.y() = static_cast<double>(p_map_resolution[1]);
      }
      if (p_map_resolution.size() >= 3) {
        resolution.z() = static_cast<double>(p_map_resolution[2]);
      }
    }
  }

  // map_offset parameter
  XmlRpc::XmlRpcValue p_map_offset;
  Point &offset = parameters_.get<Point>("map_offset");
  if (getPrivateNodeHandle().getParam("map_offset", p_map_offset)) {
    if (p_map_offset.getType() == XmlRpc::XmlRpcValue::TypeArray) {
      if (p_map_offset.size() >= 2) {
        offset.x() = static_cast<double>(p_map_offset[0]);
        offset.y() = static_cast<double>(p_map_offset[1]);
      }
      if (p_map_offset.size() >= 3) {
        offset.z() = static_cast<double>(p_map_offset[2]);
      }
    }
  }

  // map_type parameter
  std::string p_map_type = "OccupancyGridMap2D";
  getPrivateNodeHandle().getParam("map_type", p_map_type);
  map_ = MapFactory(parameters_).create<OccupancyGridMapBase>(p_map_type);
  if (!map_) {
    ROS_FATAL("Unknown map type: %s", p_map_type.c_str());
    ros::shutdown();
  }

  // get occupancy parameters
  OccupancyParameters &occupancy_parameters = parameters_.get<OccupancyParameters>("occupancy");
  double p_update_factor_free, p_update_factor_occupied;
//  private_nh_.param("update_factor_free", p_update_factor_free_, 0.4);
//  private_nh_.param("update_factor_occupied", p_update_factor_occupied_, 0.9);
  if (getPrivateNodeHandle().getParam("update_factor_free", p_update_factor_free))
    occupancy_parameters.step_free() = occupancy_parameters.getOccupancy(p_update_factor_free);
  if (getPrivateNodeHandle().getParam("update_factor_occupied", p_update_factor_occupied))
    occupancy_parameters.step_occupied() = occupancy_parameters.getOccupancy(p_update_factor_occupied);

  // get scan parameters
  ScanParameters &scan_parameters = parameters_.get<ScanParameters>("scan");
  getPrivateNodeHandle().getParam("laser_min_dist", scan_parameters.min_distance());
  getPrivateNodeHandle().getParam("laser_max_dist", scan_parameters.max_distance());
  getPrivateNodeHandle().getParam("laser_z_min_value", scan_parameters.min_z());
  getPrivateNodeHandle().getParam("laser_z_max_value", scan_parameters.max_z());

  // get other parameters
  getPrivateNodeHandle().getParam("map_frame", p_map_frame_);
  getPrivateNodeHandle().getParam("base_frame", p_base_frame_);
  getPrivateNodeHandle().getParam("odom_frame", p_odom_frame_);
  getPrivateNodeHandle().getParam("use_tf_scan_transformation", p_use_tf_scan_transformation_);
  getPrivateNodeHandle().getParam("pub_map_odom_transform_", p_pub_map_odom_transform_);
  getPrivateNodeHandle().getParam("advertise_map_service", p_advertise_map_service_);
  getPrivateNodeHandle().getParam("map_update_distance_thresh", p_map_update_translational_threshold_);
  getPrivateNodeHandle().getParam("map_update_angle_thresh", p_map_update_angular_threshold_);


//    private_nh_.param("pub_drawings", p_pub_drawings, false);
//    private_nh_.param("pub_debug_output", p_pub_debug_output_, false);
//    private_nh_.param("pub_map_odom_transform", p_pub_map_odom_transform_,true);
//    private_nh_.param("pub_odometry", p_pub_odometry_,false);
//    private_nh_.param("advertise_map_service", p_advertise_map_service_,true);
//    private_nh_.param("scan_subscriber_queue_size", p_scan_subscriber_queue_size_, 5);

//    private_nh_.param("map_resolution", p_map_resolution_, 0.025);
//    private_nh_.param("map_size", p_map_size_, 1024);
//    private_nh_.param("map_start_x", p_map_start_x_, 0.5);
//    private_nh_.param("map_start_y", p_map_start_y_, 0.5);
//    private_nh_.param("map_multi_res_levels", p_map_multi_res_levels_, 3);

//    private_nh_.param("update_factor_free", p_update_factor_free_, 0.4);
//    private_nh_.param("update_factor_occupied", p_update_factor_occupied_, 0.9);

//    private_nh_.param("map_update_distance_thresh", p_map_update_distance_threshold_, 0.4);
//    private_nh_.param("map_update_angle_thresh", p_map_update_angle_threshold_, 0.9);

//    private_nh_.param("scan_topic", p_scan_topic_, std::string("scan"));
//    private_nh_.param("sys_msg_topic", p_sys_msg_topic_, std::string("syscommand"));
//    private_nh_.param("pose_update_topic", p_pose_update_topic_, std::string("poseupdate"));

//    private_nh_.param("use_tf_scan_transformation", p_use_tf_scan_transformation_,true);
//    private_nh_.param("use_tf_pose_start_estimate", p_use_tf_pose_start_estimate_,false);
//    private_nh_.param("map_with_known_poses", p_map_with_known_poses_, false);

//    private_nh_.param("base_frame", p_base_frame_, std::string("base_link"));
//    private_nh_.param("map_frame", p_map_frame_, std::string("map"));
//    private_nh_.param("odom_frame", p_odom_frame_, std::string("odom"));

//    private_nh_.param("pub_map_scanmatch_transform", p_pub_map_scanmatch_transform_,true);
//    private_nh_.param("tf_map_scanmatch_transform_frame_name", p_tf_map_scanmatch_transform_frame_name_, std::string("scanmatcher_frame"));

//    private_nh_.param("output_timing", p_timing_output_,false);

//    private_nh_.param("map_pub_period", p_map_pub_period_, 2.0);

//    double tmp = 0.0;
//    private_nh_.param("laser_min_dist", tmp, 0.4);
//    p_sqr_laser_min_dist_ = static_cast<float>(tmp*tmp);

//    private_nh_.param("laser_max_dist", tmp, 30.0);
//    p_sqr_laser_max_dist_ = static_cast<float>(tmp*tmp);

//    private_nh_.param("laser_z_min_value", tmp, -1.0);
//    p_laser_z_min_value_ = static_cast<float>(tmp);

//    private_nh_.param("laser_z_max_value", tmp, 1.0);
//    p_laser_z_max_value_ = static_cast<float>(tmp);

  // initialize scan and scan matcher
  if (p_use_tf_scan_transformation_) scan_.setTransformer(getTransformListener(), p_base_frame_);
  matcher_ = ScanMatcher::Factory(parameters_);

  // subscribe scan
  scan_subscriber_ = getNodeHandle().subscribe<sensor_msgs::LaserScan>("scan", 10, &Node::scanCallback, this);

  // subscribe syscommand (reset)
  syscommand_subscriber_ = getNodeHandle().subscribe<std_msgs::String>("syscommand", 10, &Node::syscommandCallback, this);

  // advertise map
  map_publisher_ = getNodeHandle().advertise<nav_msgs::OccupancyGrid>("map", 1, true);
  ROS_INFO("Advertised map as %s", map_publisher_.getTopic().c_str());

  // advertise map service
  if (p_advertise_map_service_) {
    map_service_ = getNodeHandle().advertiseService("map", &Node::mapServiceCallback, this);
  }

  // advertise pose
  pose_publisher_ = getNodeHandle().advertise<geometry_msgs::PoseWithCovarianceStamped>("poseupdate", 1);

  // advertise tf
  if (p_pub_map_odom_transform_) {
    getTransformListener();
    getTransformBroadcaster();
  }

  // setup map publish timer
  double p_map_publish_period_ = 1.0;
  getPrivateNodeHandle().getParam("map_publish_period", p_map_publish_period_);
  if (p_map_publish_period_ > 0.0) map_publish_timer_ = getNodeHandle().createTimer(ros::Duration(p_map_publish_period_), boost::bind(&Node::publishMap, this));

  // reset
  reset();
}

void Node::scanCallback(const sensor_msgs::LaserScanConstPtr& scan)
{
  if (!map_ || !matcher_) return;

  // transform scan
  scan_ = scan;
  if (!scan_.valid()) return;

  // match scan
  if (!map_->empty()) {
    matcher_->match(*map_, scan_);
    if (!matcher_->valid()) return;
  }

  // update map
  float_t position_difference, orientation_difference;
  matcher_->getPoseDifference(last_map_update_pose_, position_difference, orientation_difference);
  if (map_->empty() || position_difference > p_map_update_translational_threshold_ || orientation_difference > p_map_update_angular_threshold_) {
    map_->insert(scan_, matcher_->getTransform());
    last_map_update_pose_ = matcher_->getTransform();
    last_map_update_time_ = matcher_->getStamp();
  }

  // publish pose
  publishPose();

  // publish tf
  if (p_pub_map_odom_transform_) publishTf();
}

void Node::syscommandCallback(const std_msgs::StringConstPtr& syscommand)
{
  if (syscommand->data == "reset") {
    reset();
  }
}

bool Node::mapServiceCallback(nav_msgs::GetMap::Request& request, nav_msgs::GetMap::Response& response)
{
  if (!map_) return false;
  toOccupancyGridMessage(*map_, response.map);
  return true;
}

void Node::publishMap()
{
  if (!map_) return;

  if (map_publisher_ && map_publisher_.getNumSubscribers() > 0) {
    toOccupancyGridMessage(*map_, map_message_);
    map_publisher_.publish(map_message_);
  }
}

void Node::publishPose()
{
  if (!matcher_) return;
  if (!pose_publisher_) return;
  pose_publisher_.publish(matcher_->getPoseWithCovariance());
}

void Node::publishTf()
{
  if (!matcher_ || matcher_->getStamp().isZero()) return;

  // get odom -> base transform
  tf::StampedTransform odom_base_transform_;
  try {
    getTransformListener().waitForTransform(p_base_frame_, p_odom_frame_, matcher_->getStamp(), ros::Duration(1.0));
    getTransformListener().lookupTransform(p_base_frame_, p_odom_frame_, matcher_->getStamp(), odom_base_transform_);
  } catch(tf::TransformException& e) {
    ROS_ERROR("%s", e.what());
    return;
  }

  // get map -> base transform from scan matcher
  tf::StampedTransform map_base_transform_ = matcher_->getStampedTransform();

  // publish map -> odom transform
  getTransformBroadcaster().sendTransform(tf::StampedTransform(odom_base_transform_.inverse() * map_base_transform_, matcher_->getStamp(), p_map_frame_, p_odom_frame_));
}

tf::TransformListener &Node::getTransformListener() {
  if (!tf_listener_) tf_listener_.reset(new tf::TransformListener());
  return *tf_listener_;
}

tf::TransformBroadcaster &Node::getTransformBroadcaster() {
  if (!tf_broadcaster_) tf_broadcaster_.reset(new tf::TransformBroadcaster());
  return *tf_broadcaster_;
}

} // namespace hector_mapping
