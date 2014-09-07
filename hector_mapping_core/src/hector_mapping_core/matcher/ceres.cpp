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

#include <hector_mapping_core/matcher/ceres.h>
#include <hector_mapping_core/scan.h>

#include <hector_mapping_core/map/occupancy.h>
#include <hector_mapping_core/internal/axes.h>
#include <hector_mapping_core/matcher/interpolated_map.h>
#include <hector_mapping_core/matcher/residuals.h>

#include <tf/transform_datatypes.h>

#ifdef HAVE_CERES

namespace hector_mapping {
namespace matcher {

Ceres::Ceres(const Parameters &params)
  : ScanMatcher(params)
{
}

Ceres::~Ceres()
{}

template <> struct Ceres::Solver<ScanMatcher::MATCH_2D_FIXED> {
  typedef axes::XY AxesType;
  typedef LinearInterpolatedMap<OccupancyGridMapBase, AxesType> InterpolatedMapType;

  bool operator()(const OccupancyGridMapBase &map, const Scan &scan, const ScanMatcherParameters &params, int level, const tf::Transform &initial_pose, tf::Transform &result) {
    ceres::Problem problem;
    ceres::Solver::Summary summary;

    // set solver options from params struct
    ceres::Solver::Options options;
    options.function_tolerance = params.function_tolerance();
    options.gradient_tolerance = params.gradient_tolerance();
    options.parameter_tolerance = params.parameter_tolerance();
    options.max_num_iterations = params.max_num_iterations();
    options.max_solver_time_in_seconds = params.max_solver_time_in_seconds();
    options.linear_solver_type = ceres::DENSE_QR;

    double x   = initial_pose.getOrigin().x();
    double y   = initial_pose.getOrigin().y();
    tfScalar roll, pitch, yaw;
    initial_pose.getBasis().getRPY(roll, pitch, yaw);
    double theta = yaw;

    const double initial_x   = x;
    const double initial_y   = y;
    const double initial_yaw = theta;

    InterpolatedMapType interpolated_map(map);

    for (Scan::iterator endpoint = scan.begin(); endpoint != scan.end(); endpoint++) {
      if (params.occupied_space_residual_weight() > 0) {
        problem.AddResidualBlock(
            new ceres::AutoDiffCostFunction<OccupiedSpaceResidual<InterpolatedMapType, AxesType>, 1, 1, 1, 1>(
                    new OccupiedSpaceResidual<InterpolatedMapType, AxesType>(interpolated_map, *endpoint, level)),
            new ceres::ScaledLoss(NULL, params.occupied_space_residual_weight(), ceres::TAKE_OWNERSHIP),
            &x, &y, &theta);
      }
      if (params.free_space_residual_weight() > 0) {
        problem.AddResidualBlock(
            new ceres::AutoDiffCostFunction<FreeSpaceResidual<InterpolatedMapType, AxesType>, 10, 1, 1, 1>(
                    new FreeSpaceResidual<InterpolatedMapType, AxesType>(interpolated_map, *endpoint, level, 10 /* steps */)),
            new ceres::ScaledLoss(NULL, params.free_space_residual_weight(), ceres::TAKE_OWNERSHIP),
            &x, &y, &theta);
      }
    }

//    if (params.motion_residual_weight() > 0) {
//      problem.AddResidualBlock(
//          new ceres::AutoDiffCostFunction<MotionResidual, 1, 1, 1, 1>(
//            new MotionResidual(initial_x, initial_y, initial_yaw)),
//            new ceres::ScaledLoss(NULL, params.motion_residual_weight(), ceres::TAKE_OWNERSHIP),
//          &x, &y, &theta);
//    }

    ceres::Solve(options, &problem, &summary);
    if (summary.termination_type != ceres::USER_SUCCESS) {
      ROS_ERROR("%s", summary.BriefReport().c_str());
      return false;
    }
    ROS_DEBUG("%s", summary.BriefReport().c_str());

    // set result transform
    result.getOrigin().setX(x);
    result.getOrigin().setY(y);
    tf::Matrix3x3 orientation_error; orientation_error.setRPY(0, 0, theta - initial_yaw);
    result.getBasis() = initial_pose.getBasis() * orientation_error;
    return true;
  }
};

bool Ceres::match(const OccupancyGridMapBase &map, const Scan &scan, MatchOptions options)
{
  tf::Transform result = transform_;

  switch(options) {
    case MATCH_2D_FIXED:
      if (!Solver<MATCH_2D_FIXED>()(map, scan, params_, 0, transform_, result)) return false;
      break;
    default:
      return false;
  }

  transform_.stamp_ = scan.getStamp();
  transform_.setData(result);
  return true;
}

} // namespace matcher
} // namespace hector_mapping

#endif // HAVE_CERES
