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

#include <hector_mapping2/hector_mapping.h>
#include <hector_mapping2/map/conversion.h>

#include <visualization_msgs/Marker.h>

#ifdef USE_HECTOR_TIMING
#include <hector_diagnostics/timing.h>
#endif

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
  parameters_("use_tf_pose_start_estimate", p_use_tf_pose_start_estimate_ = false);
  parameters_("pub_map_odom_transform_", p_pub_map_odom_transform_ = true);
  parameters_("advertise_map_service", p_advertise_map_service_ = true);

  parameters_("map_update_distance_thresh", p_map_update_translational_threshold_ = 0.4);
  parameters_("map_update_angle_thresh", p_map_update_angular_threshold_ = 0.9);
}

Node::~Node()
{
  map_publish_thread_.join();
}

void Node::reset()
{
  MapBase::UniqueLock lock(map_->getWriteLock());

  last_map_update_pose_.setIdentity();
  last_map_update_time_ = ros::Time();
  map_odom_transform_.setIdentity();

  if (map_) map_->reset();
  if (matcher_) matcher_->reset();

  lock.unlock();
  publishMap();
  publishPose();
}

void Node::onInit()
{
  // map_size parameter
  XmlRpc::XmlRpcValue p_map_size;
  if (getPrivateNodeHandle().getParam("map_size", p_map_size)) {
    Size &map_size = parameters_("map_size", Size(0, 0, 0));
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
  if (getPrivateNodeHandle().getParam("map_resolution", p_map_resolution)) {
    Resolution &resolution = parameters_("map_resolution", Resolution(0.0, 0.0, 0.0));
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
  if (getPrivateNodeHandle().getParam("map_offset", p_map_offset)) {
    Point &offset = parameters_("map_offset", Point(0.0, 0.0, 0.0));
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
    ROS_FATAL("Unknown map type: %s.\n\nAvailable map types:\n%s", p_map_type.c_str(), MapFactory::getMapTypes().c_str());
    ros::shutdown();
    return;
  }

  // scan matcher parameters
  ScanMatcherParameters &matcher_parameters = parameters_("matcher", ScanMatcherParameters());
  getPrivateNodeHandle().getParam("match_level_minimum", matcher_parameters.match_level_minimum());
  getPrivateNodeHandle().getParam("match_level_maximum", matcher_parameters.match_level_maximum());
  getPrivateNodeHandle().getParam("occupied_space_residual_weight", matcher_parameters.occupied_space_residual_weight());
  getPrivateNodeHandle().getParam("free_space_residual_weight", matcher_parameters.free_space_residual_weight());
  getPrivateNodeHandle().getParam("motion_residual_weight", matcher_parameters.motion_residual_weight());
  getPrivateNodeHandle().getParam("function_tolerance", matcher_parameters.function_tolerance());
  getPrivateNodeHandle().getParam("gradient_tolerance", matcher_parameters.gradient_tolerance());
  getPrivateNodeHandle().getParam("parameter_tolerance", matcher_parameters.parameter_tolerance());
  getPrivateNodeHandle().getParam("max_num_iterations", matcher_parameters.max_num_iterations());
  getPrivateNodeHandle().getParam("max_solver_time_in_seconds", matcher_parameters.max_solver_time_in_seconds());

  // get occupancy parameters
  OccupancyParameters &occupancy_parameters = parameters_("occupancy", OccupancyParameters::Default());
  double p_update_factor_free, p_update_factor_occupied;
//  private_nh_.param("update_factor_free", p_update_factor_free_, 0.4);
//  private_nh_.param("update_factor_occupied", p_update_factor_occupied_, 0.9);
  if (getPrivateNodeHandle().getParam("update_factor_free", p_update_factor_free))
    occupancy_parameters.step_free() = occupancy_parameters.getOccupancy(p_update_factor_free);
  if (getPrivateNodeHandle().getParam("update_factor_occupied", p_update_factor_occupied))
    occupancy_parameters.step_occupied() = occupancy_parameters.getOccupancy(p_update_factor_occupied);

  // get scan parameters
  ScanParameters &scan_parameters = parameters_("scan", ScanParameters());
  getPrivateNodeHandle().getParam("laser_min_dist", scan_parameters.min_distance());
  getPrivateNodeHandle().getParam("laser_max_dist", scan_parameters.max_distance());
  getPrivateNodeHandle().getParam("laser_z_min_value", scan_parameters.min_z());
  getPrivateNodeHandle().getParam("laser_z_max_value", scan_parameters.max_z());

  // get other parameters
  getPrivateNodeHandle().getParam("map_frame", p_map_frame_);
  getPrivateNodeHandle().getParam("base_frame", p_base_frame_);
  getPrivateNodeHandle().getParam("odom_frame", p_odom_frame_);
  getPrivateNodeHandle().getParam("use_tf_scan_transformation", p_use_tf_scan_transformation_);
  getPrivateNodeHandle().getParam("use_tf_pose_start_estimate", p_use_tf_pose_start_estimate_);
  getPrivateNodeHandle().getParam("pub_map_odom_transform", p_pub_map_odom_transform_);
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
  cloud_subscriber_ = getNodeHandle().subscribe<sensor_msgs::PointCloud2>("point_cloud", 10, &Node::cloudCallback, this);

  // initial pose subscriber
  initial_pose_subscriber_ = getNodeHandle().subscribe<geometry_msgs::PoseWithCovarianceStamped>("initialpose", 10, &Node::initialPoseCallback, this);

  // static map subscriber
  static_map_subscriber_ = getNodeHandle().subscribe<nav_msgs::OccupancyGrid>("static_map", 10, &Node::staticMapCallback, this);

  // subscribe syscommand (reset)
  syscommand_subscriber_ = getNodeHandle().subscribe<std_msgs::String>("syscommand", 10, &Node::syscommandCallback, this);

  // advertise map
  map_publisher_ = getNodeHandle().advertise<nav_msgs::OccupancyGrid>("map", 1);
  map_metadata_publisher_ = getNodeHandle().advertise<nav_msgs::MapMetaData>("map_metadata", 1, true);
  ROS_INFO("Advertised map as %s", map_publisher_.getTopic().c_str());

  // advertise map service
  if (p_advertise_map_service_) {
    map_service_ = getNodeHandle().advertiseService("map", &Node::mapServiceCallback, this);
  }

  // advertise pose
  pose_with_covariance_publisher_ = getNodeHandle().advertise<geometry_msgs::PoseWithCovarianceStamped>("poseupdate", 1);
  pose_publisher_ = getPrivateNodeHandle().advertise<geometry_msgs::PoseStamped>("pose", 1);
  covariance_publisher_ = getPrivateNodeHandle().advertise<visualization_msgs::Marker>("covariance", 1);

  // advertise tf
  if (p_pub_map_odom_transform_) {
    getTransformListener();
    getTransformBroadcaster();
  }

  // advertise scan cloud
  bool p_publish_scan_cloud = true;
  getPrivateNodeHandle().getParam("publish_scan_cloud", p_publish_scan_cloud);
  if (p_publish_scan_cloud) scan_.advertisePointCloud(getPrivateNodeHandle());

  // setup map publish thread
  double p_map_publish_period = 1.0;
  getPrivateNodeHandle().getParam("map_publish_period", p_map_publish_period);
  if (p_map_publish_period > 0.0) {
    map_publish_thread_ = boost::thread(boost::bind(&Node::mapPublishThread, this, ros::Rate(ros::Duration(p_map_publish_period))));
  }

  // advertise timing information
#ifdef USE_HECTOR_TIMING
  timing_publisher_ = getPrivateNodeHandle().advertise<hector_diagnostics_msgs::TimingInfo>("timing", 1);
#endif

  // reset
  reset();
}

void Node::scanCallback(const sensor_msgs::LaserScanConstPtr& scan)
{
  if (!map_ || !matcher_) return;

  // transform scan
  {
#ifdef USE_HECTOR_TIMING
    hector_diagnostics::TimingSection section("scan transformation");
#endif
    scan_ = *scan;
    if (!scan_.valid()) return;
  }

  update();
}

void Node::cloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud)
{
  if (!map_ || !matcher_) return;

  // transform scan
  {
#ifdef USE_HECTOR_TIMING
    hector_diagnostics::TimingSection section("cloud transformation");
#endif
    scan_ = *cloud;
    if (!scan_.valid()) return;
  }

  update();
}

bool Node::update()
{
  // get pose from tf
  if (p_use_tf_pose_start_estimate_) {
    tf::StampedTransform odom_pose;
    try {
      getTransformListener().waitForTransform(p_odom_frame_, p_base_frame_, scan_.getStamp(), ros::Duration(1.0));
      getTransformListener().lookupTransform(p_odom_frame_, p_base_frame_, scan_.getStamp(), odom_pose);

      matcher_->setTransform(map_odom_transform_.inverse() * odom_pose);
    } catch(tf::TransformException& e) {
      ROS_ERROR("Could not get pose from tf: %s", e.what());
      return false;
    }
  }

  // match scan
  if (!map_->empty()) {
#ifdef USE_HECTOR_TIMING
    hector_diagnostics::TimingSection section("scan matcher");
#endif
    matcher_->computeCovarianceIf(pose_with_covariance_publisher_ && pose_with_covariance_publisher_.getNumSubscribers() > 0);
    matcher_->match(*map_, scan_);
    if (!matcher_->valid()) return false;
  }

  // update map
  float_t position_difference, orientation_difference;
  matcher_->getPoseDifference(last_map_update_pose_, position_difference, orientation_difference);
  if (map_->empty() || position_difference > p_map_update_translational_threshold_ || orientation_difference > p_map_update_angular_threshold_) {
#ifdef USE_HECTOR_TIMING
    hector_diagnostics::TimingSection section("map update");
#endif
    // ros::WallTime _map_update_start = ros::WallTime::now();
      map_->insert(scan_, matcher_->getTransform());
    // ROS_DEBUG("Map update took %f seconds.", (ros::WallTime::now() - _map_update_start).toSec());

    last_map_update_pose_ = matcher_->getTransform();
    last_map_update_time_ = scan_.getStamp();
  }

  // publish pose
  publishPose();

  // publish tf
  if (p_pub_map_odom_transform_) publishTf();

  // publish timing
#ifdef USE_HECTOR_TIMING
    timing_publisher_.publish(hector_diagnostics::TimingAggregator::Instance()->update(matcher_->getStamp()));
#endif

  return true;
}

void Node::initialPoseCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr &initial_pose)
{
  tf::Pose initial_pose_tf;
  tf::poseMsgToTF(initial_pose->pose.pose, initial_pose_tf);

  // transform pose to map frame
  tf::StampedTransform pose_map_transform;
  try {
    getTransformListener().waitForTransform(p_map_frame_, initial_pose->header.frame_id, initial_pose->header.stamp, ros::Duration(1.0));
    getTransformListener().lookupTransform(p_map_frame_, initial_pose->header.frame_id, initial_pose->header.stamp, pose_map_transform);
  } catch(tf::TransformException& e) {
    ROS_ERROR("Could not transform initial pose: %s", e.what());
    return;
  }
  initial_pose_tf = pose_map_transform * initial_pose_tf;

  // test scan matcher mode
//  if (true) {
//    std::string result;
//    matcher_->evaluate(*map_, scan_, initial_pose_tf, &result);
//    ROS_INFO_STREAM(result);
//    return;
//  }

  matcher_->setInitialTransform(initial_pose_tf);
}

void Node::staticMapCallback(const nav_msgs::OccupancyGridConstPtr &static_map)
{
  ROS_ERROR("static_map feature is not implemented yet!");
  return;
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

  // Should the map always be published here?
  if (map_publisher_ && map_publisher_.getNumSubscribers() > 0) {
    if (!toOccupancyGridMessage(*map_, map_message_)) return;
    map_message_.header.stamp = ros::Time::now();
    map_publisher_.publish(map_message_);

  } else {
    // get meta data only
    if (!getMapMetaData(*map_, map_message_.info)) return;
  }

  // always publish (and latch) meta data
  map_metadata_publisher_.publish(map_message_.info);
}

void Node::publishPose()
{
  if (!matcher_) return;
  if (pose_with_covariance_publisher_ && pose_with_covariance_publisher_.getNumSubscribers() > 0)
    pose_with_covariance_publisher_.publish(matcher_->getPoseWithCovariance());
  if (pose_publisher_ && pose_publisher_.getNumSubscribers() > 0)
    pose_publisher_.publish(matcher_->getPose());
  if (covariance_publisher_ && covariance_publisher_.getNumSubscribers() > 0)
    covariance_publisher_.publish(matcher_->getCovarianceMarker());
}

void Node::publishTf()
{
  if (!matcher_ || matcher_->getStamp().isZero()) return;

  // get odom -> base transform
  tf::StampedTransform base_odom_transform;
  try {
    getTransformListener().waitForTransform(p_base_frame_, p_odom_frame_, matcher_->getStamp(), ros::Duration(1.0));
    getTransformListener().lookupTransform(p_base_frame_, p_odom_frame_, matcher_->getStamp(), base_odom_transform);
  } catch(tf::TransformException& e) {
    ROS_ERROR("Could not transform from odom frame to base frame: %s", e.what());
    return;
  }

  // get map -> base transform from scan matcher
  tf::StampedTransform map_base_transform = matcher_->getStampedTransform();

  // publish map -> odom transform
//  tf::StampedTransform map_odom_transform = tf::StampedTransform(map_base_transform * odom_base_transform.inverse(), matcher_->getStamp(), p_map_frame_, p_odom_frame_);
  map_odom_transform_ = tf::StampedTransform(map_base_transform * base_odom_transform, matcher_->getStamp(), p_map_frame_, p_odom_frame_);
  getTransformBroadcaster().sendTransform(map_odom_transform_);
}

tf::TransformListener &Node::getTransformListener() {
  if (!tf_listener_) tf_listener_.reset(new tf::TransformListener());
  return *tf_listener_;
}

tf::TransformBroadcaster &Node::getTransformBroadcaster() {
  if (!tf_broadcaster_) tf_broadcaster_.reset(new tf::TransformBroadcaster());
  return *tf_broadcaster_;
}

void Node::mapPublishThread(ros::Rate rate)
{
  while(ros::ok()) {
    {
#ifdef USE_HECTOR_TIMING
      hector_diagnostics::TimingSection section("map publish");
#endif
      publishMap();
    }
    rate.sleep();
  }
}

} // namespace hector_mapping
