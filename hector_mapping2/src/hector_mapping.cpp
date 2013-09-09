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
{
}

Node::~Node()
{

}

void Node::onInit()
{
  // initialize map
  OccupancyGridMap::Parameters parameters;

  XmlRpc::XmlRpcValue p_map_size;
  if (getPrivateNodeHandle().getParam("map_size", p_map_size)) {
    if (p_map_size.getType() == XmlRpc::XmlRpcValue::TypeInt) {
      parameters.size().x() = parameters.size().y() = static_cast<int>(p_map_size);
    } else if (p_map_size.getType() == XmlRpc::XmlRpcValue::TypeArray) {
      if (p_map_size.size() >= 2) {
        parameters.size().x() = static_cast<int>(p_map_size[0]);
        parameters.size().y() = static_cast<int>(p_map_size[1]);
      }
      if (p_map_size.size() >= 3) {
        parameters.size().z() = static_cast<int>(p_map_size[2]);
      }
    }
  }

  XmlRpc::XmlRpcValue p_map_resolution;
  if (getPrivateNodeHandle().getParam("map_resolution", p_map_resolution)) {
    if (p_map_resolution.getType() == XmlRpc::XmlRpcValue::TypeInt) {
      parameters.resolution().x() = parameters.resolution().y() = static_cast<int>(p_map_resolution);
    } else if (p_map_size.getType() == XmlRpc::XmlRpcValue::TypeArray) {
      if (p_map_resolution.size() >= 2) {
        parameters.resolution().x() = static_cast<double>(p_map_resolution[0]);
        parameters.resolution().y() = static_cast<double>(p_map_resolution[1]);
      }
      if (p_map_resolution.size() >= 3) {
        parameters.resolution().z() = static_cast<double>(p_map_resolution[2]);
      }
    }
  }

  std::string p_map_type = "OccupancyGridMap2D";
  getPrivateNodeHandle().getParam("map_type", p_map_type);
  if (p_map_type == "OccupancyGridMap2D") {
    map_.reset(new OccupancyGridMap2D(parameters));
  } else if (p_map_type == "OccupancyQuadTreeMap2D") {
    map_.reset(new OccupancyQuadTreeMap2D(parameters));
  } else {
    ROS_FATAL("Unknown map type: %s", p_map_type.c_str());
    ros::shutdown();
  }

  // initialize scan matcher
  matcher_.reset(new ScanMatcher());

  // subscribe scan
  scan_subscriber_ = getNodeHandle().subscribe<sensor_msgs::LaserScan>("scan", 10, &Node::scanCallback, this);

  // publish map
  map_publisher_ = getNodeHandle().advertise<nav_msgs::OccupancyGrid>("map", 1, true);
  ROS_INFO("Advertising map as %s", map_publisher_.getTopic().c_str());

  double p_map_publish_period_ = 1.0;
  getPrivateNodeHandle().getParam("map_publish_period", p_map_publish_period_);
  if (p_map_publish_period_ > 0.0) map_publish_timer_ = getNodeHandle().createTimer(ros::Duration(p_map_publish_period_), boost::bind(&Node::publishMap, this));
}

void Node::scanCallback(const sensor_msgs::LaserScanConstPtr& scan)
{
  if (!map_ || !matcher_) return;

  // match scan
  scan_ = scan;
  matcher_->match(*map_, scan_);

  // update map
  map_->insertScan(scan_, matcher_->getTransform());

  // publish pose
  matcher_->getPoseWithCovariance(pose_message_);
  pose_publisher_.publish(pose_message_);
}

void Node::publishMap()
{
  if (!map_) return;

  if (map_publisher_ && map_publisher_.getNumSubscribers() > 0) {
    toOccupancyGridMessage(*map_, map_message_);
    map_publisher_.publish(map_message_);
  }
}


} // namespace hector_mapping
