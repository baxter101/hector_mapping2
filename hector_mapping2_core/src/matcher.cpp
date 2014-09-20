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

#include <hector_mapping2_core/matcher.h>
#include <hector_mapping2_core/matcher/ceres.h>
#include <hector_mapping2_core/util/marker_conversion.h>

namespace hector_mapping {

ScanMatcherParameters::ScanMatcherParameters()
  : match_level_minimum_(0),
    match_level_maximum_(0),
    occupied_space_residual_weight_(1.0),
    free_space_residual_weight_(0.0),
    motion_residual_weight_(0.0),
    function_tolerance_(1e-3),
    gradient_tolerance_(1e-10),
    parameter_tolerance_(1e-8),
    max_num_iterations_(50),
    max_solver_time_in_seconds_(0.05)
{}


ScanMatcher::ScanMatcher(const Parameters& _params)
  : covariance_enabled_(false)
  , covariance_valid_(false)
{
  _params.add("matcher", params_);
  transform_.frame_id_ = _params.get<std::string>("map_frame");
  transform_.child_frame_id_ = _params.get<std::string>("base_frame");
  reset();
}

ScanMatcher::~ScanMatcher()
{}

ScanMatcherPtr ScanMatcher::Factory(const Parameters& params)
{
#ifdef HAVE_CERES
  return ScanMatcherPtr(new matcher::Ceres(params));
#else
  return ScanMatcherPtr();
#endif
}

void ScanMatcher::reset()
{
  transform_.stamp_ = ros::Time();
  transform_.setIdentity();
//  transform_.header.stamp = ros::Time();
//  transform_.transform = Transform();
//  transform_.transform.rotation.w = 1.0;
}

bool ScanMatcher::valid() const
{
  return true;
}

void ScanMatcher::getPoseDifference(const tf::Transform& other, float_t& position_difference, float_t& orientation_difference) const {
  position_difference = getTransform().getOrigin().distance(other.getOrigin());
  orientation_difference = getTransform().getRotation().angleShortestPath(other.getRotation());
}

void ScanMatcher::computeCovarianceIf(bool enabled) {
  covariance_enabled_ = enabled;
}

void ScanMatcher::getPose(geometry_msgs::Pose& pose) const {
  tf::pointTFToMsg(transform_.getOrigin(), pose.position);
  tf::quaternionTFToMsg(transform_.getRotation(),  pose.orientation);
}

void ScanMatcher::getPose(geometry_msgs::PoseStamped& pose) const {
  pose.header.stamp = transform_.stamp_;
  pose.header.frame_id = transform_.frame_id_;
  getPose(pose.pose);
}

void ScanMatcher::getPose(geometry_msgs::PoseWithCovarianceStamped& pose) const {
  pose.header.stamp = transform_.stamp_;
  pose.header.frame_id = transform_.frame_id_;
  getPose(pose.pose.pose);

  if (covariance_valid_)
    std::copy(&(covariance_(0,0)), &(covariance_(5,5)) + 1, pose.pose.covariance.begin());
  else
    pose.pose.covariance.assign(0.0);
}

geometry_msgs::PoseStamped ScanMatcher::getPose() const {
  geometry_msgs::PoseStamped pose;
  getPose(pose);
  return pose;
}

geometry_msgs::PoseWithCovarianceStamped ScanMatcher::getPoseWithCovariance() const {
  geometry_msgs::PoseWithCovarianceStamped pose_with_covariance;
  getPose(pose_with_covariance);
  return pose_with_covariance;
}

visualization_msgs::Marker ScanMatcher::getCovarianceMarker() const {
  visualization_msgs::Marker marker;

  marker.ns = "covariance";
  marker.color.r = 0.0;
  marker.color.g = 0.0;
  marker.color.b = 1.0;
  marker.color.a = 0.5;

  if (covariance_valid_) {
    covarianceToMarker(transform_, covariance_.block<3,3>(0,0), marker, /* invert = */ true);
  } else {
    marker.action = visualization_msgs::Marker::DELETE;
  }
  return marker;
}

} // namespace hector_mapping

