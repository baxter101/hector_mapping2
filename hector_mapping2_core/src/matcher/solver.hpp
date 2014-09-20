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

#ifndef HECTOR_MAPPING2_MATCHER_SOLVER_HPP
#define HECTOR_MAPPING2_MATCHER_SOLVER_HPP

#ifndef NDEBUG
  // define NDEBUG (todo: check if this really improves speed)
  // #define NDEBUG
#endif // NDEBUG

#include <hector_mapping2_core/map/occupancy.h>
#include <hector_mapping2_core/internal/axes.h>

#include <tf/transform_datatypes.h>

#include "interpolated_map.hpp"
#include "residuals.hpp"

namespace hector_mapping {
namespace matcher {

class Ceres::Solver {
private:
  template <ScanMatcher::MatchType> struct Types;
  boost::shared_ptr<void> interpolated_map_;

  ceres::Problem *problem_;
  ceres::Solver::Options options_;
  ceres::Solver::Summary summary_;

  double xy_[2], z_[1], rollpitch_[2], yaw_[1];

  bool evaluated_;
  double cost_;
  std::vector<double> residuals_;
  std::vector<double> gradient_;
  Eigen::MatrixXd jacobian_;

  Solver(const ScanMatcherParameters &params, const tf::Transform &initial_pose);

public:
  template <ScanMatcher::MatchType>
  static SolverPtr initialize(const OccupancyGridMapBase &map, const Scan &scan, const ScanMatcherParameters &params, const tf::Transform &initial_pose, int level = 0);
  ~Solver();

  bool solve(tf::Transform &result);
  double evaluate();

  std::string evaluateDebug();

  bool getCovariance(geometry_msgs::PoseWithCovariance::_covariance_type& matrix);
  bool getInformationMatrix(geometry_msgs::PoseWithCovariance::_covariance_type& matrix);

  template <typename MatrixType> bool getInformationMatrix(MatrixType& matrix);
  template <typename MatrixType> bool getCovariance(MatrixType& matrix);
};

template <> struct Ceres::Solver::Types<ScanMatcher::MATCH_2D>
{
  typedef axes::XY Axes;
  typedef LinearInterpolatedMap<OccupancyGridMapBase, Axes> InterpolatedMap;
};

template <> struct Ceres::Solver::Types<ScanMatcher::MATCH_3D>
{
  typedef axes::XYZ Axes;
  typedef LinearInterpolatedMap<OccupancyGridMapBase, Axes> InterpolatedMap;
};

Ceres::Solver::Solver(const ScanMatcherParameters &params, const tf::Transform &initial_pose)
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
  xy_[0] = initial_pose.getOrigin().x();
  xy_[1] = initial_pose.getOrigin().y();
  z_[0]  = initial_pose.getOrigin().z();
  tfScalar roll, pitch, yaw;
  initial_pose.getBasis().getRPY(roll, pitch, yaw);
  rollpitch_[0] = roll;
  rollpitch_[1] = pitch;
  yaw_[0]        = yaw;

  // create problem :-)
  problem_ = new ceres::Problem;
  problem_->AddParameterBlock(xy_, 2);
  problem_->AddParameterBlock(z_, 1);
  problem_->AddParameterBlock(rollpitch_, 2);
  problem_->AddParameterBlock(yaw_, 1);

  evaluated_ = false;
}

Ceres::Solver::~Solver()
{
  delete problem_;
}

template <ScanMatcher::MatchType matchtype>
Ceres::SolverPtr Ceres::Solver::initialize(const OccupancyGridMapBase &_map, const Scan &scan, const ScanMatcherParameters &params, const tf::Transform &initial_pose, int level)
{
  SolverPtr solver(new Solver(params, initial_pose));

  typedef Types<matchtype> types;
  typedef typename types::InterpolatedMap InterpolatedMap;
  typedef typename types::Axes Axes;

  InterpolatedMap *map(new InterpolatedMap(_map));
  solver->interpolated_map_.reset(map);

  // add parameter blocks
  if (matchtype == ScanMatcher::MATCH_2D) {
    solver->problem_->SetParameterBlockConstant(solver->z_);
    solver->problem_->SetParameterBlockConstant(solver->rollpitch_);
  }

  // add residuals
  for (Scan::iterator endpoint = scan.begin(); endpoint != scan.end(); endpoint++) {
    if (params.occupied_space_residual_weight() > 0) {
      solver->problem_->AddResidualBlock(
          new ceres::AutoDiffCostFunction<OccupiedSpaceResidual<InterpolatedMap>, 1, 2, 1, 2, 1>(
                  new OccupiedSpaceResidual<InterpolatedMap>(*map, *endpoint, level)),
          new ceres::ScaledLoss(NULL, params.occupied_space_residual_weight(), ceres::TAKE_OWNERSHIP),
          solver->xy_, solver->z_, solver->rollpitch_, solver->yaw_);
    }
    if (params.free_space_residual_weight() > 0) {
      solver->problem_->AddResidualBlock(
          new ceres::AutoDiffCostFunction<FreeSpaceResidual<InterpolatedMap>, 10, 2, 1, 2, 1>(
                  new FreeSpaceResidual<InterpolatedMap>(*map, *endpoint, level, 10 /* steps */)),
          new ceres::ScaledLoss(NULL, params.free_space_residual_weight(), ceres::TAKE_OWNERSHIP),
            solver->xy_, solver->z_, solver->rollpitch_, solver->yaw_);
    }
  }

//    if (params.motion_residual_weight() > 0) {
//      problem.AddResidualBlock(
//          new ceres::AutoDiffCostFunction<MotionResidual, 2, 1, 2, 1>(
//            new MotionResidual(initial_x, initial_y, initial_yaw)),
//            new ceres::ScaledLoss(NULL, params.motion_residual_weight(), ceres::TAKE_OWNERSHIP),
//             solver->xy_, solver->z_, solver->rollpitch_, solver->yaw_);
//    }

  return solver;
}

bool Ceres::Solver::solve(tf::Transform &result) {
  evaluated_ = false;

  ceres::Solve(options_, problem_, &summary_);
  ROS_DEBUG("%s", summary_.FullReport().c_str());
  if (summary_.termination_type != ceres::USER_SUCCESS &&
      summary_.termination_type != ceres::FUNCTION_TOLERANCE) {
    ROS_ERROR("%s", summary_.BriefReport().c_str());
    // return false;
  }

  // set result transform
  result.getOrigin().setValue(xy_[0], xy_[1], z_[0]);
  result.getBasis().setRPY(rollpitch_[0], rollpitch_[1], yaw_[0]);
  return true;
}

double Ceres::Solver::evaluate() {
  ceres::CRSMatrix crs_jacobian;
  if (!problem_->Evaluate(ceres::Problem::EvaluateOptions(), &cost_, &residuals_, &gradient_, &crs_jacobian))
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

std::string Ceres::Solver::evaluateDebug() {
  if (!evaluated_) evaluate();
  std::stringstream s;
  Eigen::Map<Eigen::MatrixXd> r(residuals_.data(), residuals_.size(), 1);
  Eigen::Map<Eigen::MatrixXd> g(gradient_.data(), 1, gradient_.size());

  s << "Evaluation at [ " << xy_[0] << " " << xy_[1] << " " << yaw_[0] << " ]:" << std::endl
    << "  cost = " << cost_ << std::endl
    << "  residuals = " << r.transpose() << std::endl
    << "  gradient = " << g << std::endl
    << "  jacobian = " << jacobian_ << std::endl
    << "  hessian = " << (jacobian_.transpose() * jacobian_) << std::endl
    // << "  gradient = " << (J.transpose() * r) << std::endl
    ;

  return s.str();
}

bool Ceres::Solver::getCovariance(geometry_msgs::PoseWithCovariance::_covariance_type& matrix) {
  Eigen::Map<Eigen::Matrix<geometry_msgs::PoseWithCovariance::_covariance_type::value_type,6,6> > map(matrix.data());
  return getCovariance(map);
}

bool Ceres::Solver::getInformationMatrix(geometry_msgs::PoseWithCovariance::_covariance_type& matrix) {
  Eigen::Map<Eigen::Matrix<geometry_msgs::PoseWithCovariance::_covariance_type::value_type,6,6> > map(matrix.data());
  return getInformationMatrix(map);
}

template <typename MatrixType>
bool Ceres::Solver::getInformationMatrix(MatrixType& matrix) {
//    if (!getCovariance(matrix)) return false;
//    matrix = matrix.inverse();
//    return true;

  if (!evaluated_ && evaluate() < 0.0) return false;
  Eigen::Matrix<double,6,6> I = jacobian_.transpose() * jacobian_;

  // fill output matrix
//  matrix.resize(6,6);
//  matrix = -MatrixType::Identity(6,6);
//  matrix(0,0) = I(0,0);
//  matrix(0,1) = I(0,1);
//  matrix(0,5) = I(0,2);
//  matrix(1,0) = I(1,0);
//  matrix(1,1) = I(1,1);
//  matrix(1,5) = I(1,2);
//  matrix(5,0) = I(2,0);
//  matrix(5,1) = I(2,1);
//  matrix(5,5) = I(2,2);

  matrix = I / 1e4;
  return true;
}

template <typename MatrixType>
bool Ceres::Solver::getCovariance(MatrixType& matrix) {
  ceres::Covariance::Options options;
  ceres::Covariance covariance(options);
  std::vector< std::pair<const double *, const double *> > covariance_blocks(10);
  covariance_blocks[0] = std::make_pair<const double *, const double *>(xy_, xy_);
  covariance_blocks[1] = std::make_pair<const double *, const double *>(xy_, z_);
  covariance_blocks[2] = std::make_pair<const double *, const double *>(xy_, rollpitch_);
  covariance_blocks[3] = std::make_pair<const double *, const double *>(xy_, yaw_);
  covariance_blocks[4] = std::make_pair<const double *, const double *>(z_, z_);
  covariance_blocks[5] = std::make_pair<const double *, const double *>(z_, rollpitch_);
  covariance_blocks[6] = std::make_pair<const double *, const double *>(z_, yaw_);
  covariance_blocks[7] = std::make_pair<const double *, const double *>(rollpitch_, rollpitch_);
  covariance_blocks[8] = std::make_pair<const double *, const double *>(rollpitch_, yaw_);
  covariance_blocks[9] = std::make_pair<const double *, const double *>(yaw_, yaw_);
  if (!covariance.Compute(covariance_blocks, problem_)) return false;

  // fill output matrix
  matrix.resize(6,6);
  matrix = -MatrixType::Identity(6,6);
//  covariance.GetCovarianceBlock(x_, x_, &matrix(0,0));
//  covariance.GetCovarianceBlock(x_, y_, &matrix(0,1));
//  covariance.GetCovarianceBlock(x_, theta_, &matrix(0,5));
//  covariance.GetCovarianceBlock(y_, y_, &matrix(1,1));
//  covariance.GetCovarianceBlock(y_, theta_, &matrix(1,5));
//  covariance.GetCovarianceBlock(theta_, theta_, &matrix(5,5));
//  matrix(1,0) = matrix(0,1);
//  matrix(5,0) = matrix(0,5);
//  matrix(5,1) = matrix(1,5);

  Eigen::Matrix<double,2,2> C_xy_xy;
  Eigen::Matrix<double,2,1> C_xy_z;
  Eigen::Matrix<double,2,2> C_xy_rollpitch;
  Eigen::Matrix<double,2,1> C_xy_yaw;
  Eigen::Matrix<double,1,1> C_z_z;
  Eigen::Matrix<double,1,2> C_z_rollpitch;
  Eigen::Matrix<double,1,1> C_z_yaw;
  Eigen::Matrix<double,2,2> C_rollpitch_rollpitch;
  Eigen::Matrix<double,2,1> C_rollpitch_yaw;
  Eigen::Matrix<double,1,1> C_yaw_yaw;

  covariance.GetCovarianceBlock(xy_,        xy_,        &C_xy_xy.coeffRef(0));
  covariance.GetCovarianceBlock(xy_,        z_,         &C_xy_z.coeffRef(0));
  covariance.GetCovarianceBlock(xy_,        rollpitch_, &C_xy_rollpitch.coeffRef(0));
  covariance.GetCovarianceBlock(xy_,        yaw_,       &C_xy_yaw.coeffRef(0));
  covariance.GetCovarianceBlock(z_,         z_,         &C_z_z.coeffRef(0));
  covariance.GetCovarianceBlock(z_,         rollpitch_, &C_z_rollpitch.coeffRef(0));
  covariance.GetCovarianceBlock(z_,         yaw_,       &C_z_yaw.coeffRef(0));
  covariance.GetCovarianceBlock(rollpitch_, rollpitch_, &C_rollpitch_rollpitch.coeffRef(0));
  covariance.GetCovarianceBlock(rollpitch_, yaw_,       &C_rollpitch_yaw.coeffRef(0));
  covariance.GetCovarianceBlock(yaw_,       yaw_,       &C_yaw_yaw.coeffRef(0));

  matrix << C_xy_xy,                    C_xy_z,                    C_xy_rollpitch,              C_xy_yaw,
            C_xy_z.transpose(),         C_z_z,                     C_z_rollpitch,               C_z_yaw,
            C_xy_rollpitch.transpose(), C_z_rollpitch.transpose(), C_rollpitch_rollpitch,       C_rollpitch_yaw,
            C_xy_yaw.transpose(),       C_z_yaw.transpose(),       C_rollpitch_yaw.transpose(), C_yaw_yaw;
  matrix *= 1e4;

  return true;
}

} // namespace matcher
} // namespace hector_mapping

#endif // HECTOR_MAPPING2_MATCHER_SOLVER_HPP
