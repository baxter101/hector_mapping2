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

#ifndef NDEBUG
  // define NDEBUG (todo: check if this really improves speed)
  // #define NDEBUG
#endif // NDEBUG

#include <hector_mapping_core/matcher/ceres.h>
#include <hector_mapping_core/scan.h>

#include <hector_mapping_core/map/occupancy.h>
#include <hector_mapping_core/internal/axes.h>

#include <tf/transform_datatypes.h>

#include "interpolated_map.hpp"
#include "residuals.hpp"

#ifdef HAVE_CERES

namespace hector_mapping {
namespace matcher {

Ceres::Ceres(const Parameters &params)
  : ScanMatcher(params)
{
}

Ceres::~Ceres()
{}

template <> class Ceres::Solver<ScanMatcher::MATCH_2D_FIXED> {
public:
  typedef axes::XY AxesType;
  typedef LinearInterpolatedMap<OccupancyGridMapBase, AxesType> InterpolatedMapType;

  Solver(const OccupancyGridMapBase &map, const Scan &scan, const ScanMatcherParameters &params, const tf::Transform &initial_pose, int level = 0)
    : map_(map), initial_pose_(initial_pose), evaluated_(false)
  {
    // set solver options from params struct
    options_.function_tolerance = params.function_tolerance();
    options_.gradient_tolerance = params.gradient_tolerance();
    options_.parameter_tolerance = params.parameter_tolerance();
    options_.max_num_iterations = params.max_num_iterations();
    options_.max_solver_time_in_seconds = /* 1e3; */ params.max_solver_time_in_seconds();
    options_.linear_solver_type = ceres::DENSE_QR;
    options_.preconditioner_type = ceres::JACOBI;
//    options_.use_nonmonotonic_steps = true;

//    options_.minimizer_type = ceres::LINE_SEARCH;
//    options_.line_search_direction_type = ceres::STEEPEST_DESCENT;

    // initialize solver variables
    *x_ = initial_pose_.getOrigin().x();
    *y_ = initial_pose_.getOrigin().y();
    tfScalar roll, pitch, yaw;
    initial_pose_.getBasis().getRPY(roll, pitch, yaw);
    *theta_ = yaw;

    // add residuals
    for (Scan::iterator endpoint = scan.begin(); endpoint != scan.end(); endpoint++) {
      if (params.occupied_space_residual_weight() > 0) {
        problem_.AddResidualBlock(
            new ceres::AutoDiffCostFunction<OccupiedSpaceResidual<InterpolatedMapType, AxesType>, 1, 1, 1, 1>(
                    new OccupiedSpaceResidual<InterpolatedMapType, AxesType>(map_, *endpoint, level)),
            new ceres::ScaledLoss(NULL, params.occupied_space_residual_weight(), ceres::TAKE_OWNERSHIP),
            x_, y_, theta_);
      }
      if (params.free_space_residual_weight() > 0) {
        problem_.AddResidualBlock(
            new ceres::AutoDiffCostFunction<FreeSpaceResidual<InterpolatedMapType, AxesType>, 10, 1, 1, 1>(
                    new FreeSpaceResidual<InterpolatedMapType, AxesType>(map_, *endpoint, level, 10 /* steps */)),
            new ceres::ScaledLoss(NULL, params.free_space_residual_weight(), ceres::TAKE_OWNERSHIP),
            x_, y_, theta_);
      }
    }

//    if (params.motion_residual_weight() > 0) {
//      problem.AddResidualBlock(
//          new ceres::AutoDiffCostFunction<MotionResidual, 1, 1, 1, 1>(
//            new MotionResidual(initial_x, initial_y, initial_yaw)),
//            new ceres::ScaledLoss(NULL, params.motion_residual_weight(), ceres::TAKE_OWNERSHIP),
//          &x, &y, &theta);
//    }
  }

  bool solve(tf::Transform &result) {
    evaluated_ = false;

    // remember initial values to compute updated transform at the end
    const double initial_x   = *x_;
    const double initial_y   = *y_;
    const double initial_yaw = *theta_;

    ceres::Solve(options_, &problem_, &summary_);
    ROS_DEBUG("%s", summary_.FullReport().c_str());
    if (summary_.termination_type != ceres::USER_SUCCESS &&
        summary_.termination_type != ceres::FUNCTION_TOLERANCE) {
      ROS_ERROR("%s", summary_.BriefReport().c_str());
      // return false;
    }

    // set result transform
    result.getOrigin().setX(*x_);
    result.getOrigin().setY(*y_);
    tf::Matrix3x3 orientation_error; orientation_error.setRPY(0, 0, *theta_ - initial_yaw);
    result.getBasis() = initial_pose_.getBasis() * orientation_error;
    return true;
  }

  double evaluate() {
    ceres::CRSMatrix crs_jacobian;
    if (!problem_.Evaluate(ceres::Problem::EvaluateOptions(), &cost_, &residuals_, &gradient_, &crs_jacobian))
      return -1.0;

    evaluated_ = true;
    jacobian_ = Eigen::MatrixXd::Zero(crs_jacobian.num_rows, crs_jacobian.num_cols);
    std::vector<int>::const_iterator next_row = crs_jacobian.rows.begin();
    int row = *next_row++;
    for(int i = 0; i < crs_jacobian.cols.size(); ++i) {
      if (next_row != crs_jacobian.rows.end() && i == *next_row) { row++; next_row++; }
      int col = crs_jacobian.cols[i];
      jacobian_(row,col) = crs_jacobian.values[i];
    }
    return cost_;
  }

  std::string evaluateDebug() {
    if (!evaluated_) evaluate();
    std::stringstream s;
    Eigen::Map<Eigen::MatrixXd> r(residuals_.data(), residuals_.size(), 1);
    Eigen::Map<Eigen::MatrixXd> g(gradient_.data(), 1, gradient_.size());

    s << "Evaluation at [ " << *x_ << " " << *y_ << " " << *theta_ << " ]:" << std::endl
      << "  cost = " << cost_ << std::endl
      << "  residuals = " << r.transpose() << std::endl
      << "  gradient = " << g << std::endl
      << "  jacobian = " << jacobian_ << std::endl
      << "  hessian = " << (jacobian_.transpose() * jacobian_) << std::endl
      // << "  gradient = " << (J.transpose() * r) << std::endl
      ;

    return s.str();
  }

  bool getCovariance(geometry_msgs::PoseWithCovariance::_covariance_type& matrix) {
    Eigen::Map<Eigen::Matrix<geometry_msgs::PoseWithCovariance::_covariance_type::value_type,6,6> > map(matrix.data());
    return getCovariance(map);
  }

  bool getInformationMatrix(geometry_msgs::PoseWithCovariance::_covariance_type& matrix) {
    Eigen::Map<Eigen::Matrix<geometry_msgs::PoseWithCovariance::_covariance_type::value_type,6,6> > map(matrix.data());
    return getInformationMatrix(map);
  }

  template <typename MatrixType> bool getInformationMatrix(MatrixType& matrix) {
//    if (!getCovariance(matrix)) return false;
//    matrix = matrix.inverse();
//    return true;

    if (!evaluated_ && evaluate() < 0.0) return false;
    Eigen::Matrix<double,3,3> I = jacobian_.transpose() * jacobian_;

    // fill output matrix
    matrix.resize(6,6);
    matrix = -MatrixType::Identity(6,6);
    matrix(0,0) = I(0,0);
    matrix(0,1) = I(0,1);
    matrix(0,5) = I(0,2);
    matrix(1,0) = I(1,0);
    matrix(1,1) = I(1,1);
    matrix(1,5) = I(1,2);
    matrix(5,0) = I(2,0);
    matrix(5,1) = I(2,1);
    matrix(5,5) = I(2,2);
    return true;
  }

  template <typename MatrixType> bool getCovariance(MatrixType& matrix) {
    ceres::Covariance::Options options;
    ceres::Covariance covariance(options);
    std::vector< std::pair<const double *, const double *> > covariance_blocks(6);
    covariance_blocks[0] = std::make_pair<const double *, const double *>(x_, x_);
    covariance_blocks[1] = std::make_pair<const double *, const double *>(x_, y_);
    covariance_blocks[2] = std::make_pair<const double *, const double *>(x_, theta_);
    covariance_blocks[3] = std::make_pair<const double *, const double *>(y_, y_);
    covariance_blocks[4] = std::make_pair<const double *, const double *>(y_, theta_);
    covariance_blocks[5] = std::make_pair<const double *, const double *>(theta_, theta_);
    if (!covariance.Compute(covariance_blocks, &problem_)) return false;

    // fill output matrix
    matrix.resize(6,6);
    matrix = -MatrixType::Identity(6,6);
    covariance.GetCovarianceBlock(x_, x_, &matrix(0,0));
    covariance.GetCovarianceBlock(x_, y_, &matrix(0,1));
    covariance.GetCovarianceBlock(x_, theta_, &matrix(0,5));
    covariance.GetCovarianceBlock(y_, y_, &matrix(1,1));
    covariance.GetCovarianceBlock(y_, theta_, &matrix(1,5));
    covariance.GetCovarianceBlock(theta_, theta_, &matrix(5,5));
    matrix(1,0) = matrix(0,1);
    matrix(5,0) = matrix(0,5);
    matrix(5,1) = matrix(1,5);
    return true;
  }

private:
  ceres::Problem problem_;
  ceres::Solver::Options options_;
  ceres::Solver::Summary summary_;

  InterpolatedMapType map_;
  tf::Transform initial_pose_;
  double x_[1], y_[1], theta_[1];

  bool evaluated_;
  double cost_;
  std::vector<double> residuals_;
  std::vector<double> gradient_;
  Eigen::MatrixXd jacobian_;
};

bool Ceres::match(const OccupancyGridMapBase &map, const Scan &scan, MatchType type)
{
  // acquire a shared lock
  MapBase::SharedLock lock(map.getLock());
  tf::Transform result = transform_;
  covariance_valid_ = false;

  switch(type) {
    case MATCH_2D_FIXED:
      {
        Solver<MATCH_2D_FIXED> solver(map, scan, params_, transform_);
        if (!solver.solve(result)) return false;
        // covariance_valid_ = solver.getCovariance(covariance_);
        covariance_valid_ = solver.getInformationMatrix(covariance_);
      }
      break;

    default:
      return false;
  }

  transform_.stamp_ = scan.getStamp();
  transform_.setData(result);
  return true;
}

double Ceres::evaluate(const OccupancyGridMapBase &map, const Scan &scan, const tf::Transform &pose, std::string *resultString, MatchType type) const
{
  // acquire a shared lock
  MapBase::SharedLock lock(map.getLock());
  double cost;

  switch(type) {
    case MATCH_2D_FIXED:
      {
        Solver<MATCH_2D_FIXED> solver(map, scan, params_, pose);
        cost = solver.evaluate();
        if (resultString) *resultString = solver.evaluateDebug();
      }
      break;
    default:
      return -1.0;
  }

  return cost;
}

} // namespace matcher
} // namespace hector_mapping

#endif // HAVE_CERES
