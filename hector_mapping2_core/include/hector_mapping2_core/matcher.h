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


#ifndef HECTOR_MAPPING2_SCAN_MATCHER_H
#define HECTOR_MAPPING2_SCAN_MATCHER_H

#include <hector_mapping2_core/types.h>
#include <hector_mapping2_core/parameters.h>
#include <hector_mapping2_core/internal/macros.h>

#include <tf/transform_datatypes.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <visualization_msgs/Marker.h>

namespace hector_mapping {

struct ScanMatcherParameters {
  PARAMETER(ScanMatcherParameters, int, match_level_minimum);
  PARAMETER(ScanMatcherParameters, int, match_level_maximum);
  PARAMETER(ScanMatcherParameters, double, occupied_space_residual_weight);
  PARAMETER(ScanMatcherParameters, double, free_space_residual_weight);
  PARAMETER(ScanMatcherParameters, double, motion_residual_weight);
  PARAMETER(ScanMatcherParameters, double, function_tolerance);
  PARAMETER(ScanMatcherParameters, double, gradient_tolerance);
  PARAMETER(ScanMatcherParameters, double, parameter_tolerance);
  PARAMETER(ScanMatcherParameters, int, max_num_iterations);
  PARAMETER(ScanMatcherParameters, double, max_solver_time_in_seconds);

public:
  ScanMatcherParameters();
};

class ScanMatcher
{
public:
  static ScanMatcherPtr Factory(const Parameters& params = Parameters());
  virtual ~ScanMatcher();

  virtual void reset();
  virtual bool valid() const;

  // update scan matcher
  enum MatchType { MATCH_2D, MATCH_3D };
  virtual bool match(const OccupancyGridMapBase& map, const Scan& scan, MatchType type = MATCH_2D) = 0;

  // test match
  virtual double evaluate(const OccupancyGridMapBase& map, const Scan& scan, const tf::Transform& pose, std::string *resultString = 0, MatchType type = MATCH_2D) const = 0;

  // set/get transform
  void setInitialTransform(const tf::Transform& transform) { transform_.setData(transform); transform_.stamp_ = ros::Time(); }
  void setTransform(const tf::Transform& transform) { transform_.setData(transform); }
  const ros::Time& getStamp() const { return transform_.stamp_; }
  const tf::StampedTransform& getStampedTransform() const { return transform_; }
  const tf::Transform& getTransform() const { return transform_; }
  void getPoseDifference(const tf::Transform& transform, float_t& position_difference, float_t& orientation_difference) const;

  // get pose with covariance
  void computeCovarianceIf(bool enabled);
  void getPose(geometry_msgs::Pose& pose) const;
  void getPose(geometry_msgs::PoseStamped& pose) const;
  void getPose(geometry_msgs::PoseWithCovarianceStamped& pose) const;
  geometry_msgs::PoseStamped getPose() const;
  geometry_msgs::PoseWithCovarianceStamped getPoseWithCovariance() const;
  visualization_msgs::Marker getCovarianceMarker() const;

protected:
  ScanMatcher(const Parameters& params = Parameters());
  ScanMatcherParameters params_;

  bool valid_;
  tf::StampedTransform transform_;

  // do not use float_t here because of ceres
  Eigen::Matrix<double,6,6> covariance_;
  bool covariance_enabled_;
  bool covariance_valid_;
};

} // namespace hector_mapping

#endif // HECTOR_MAPPING2_SCAN_MATCHER_H
