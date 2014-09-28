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


#ifndef HECTOR_MAPPING2_MATCHER_RESIDUALS_H
#define HECTOR_MAPPING2_MATCHER_RESIDUALS_H

#include <hector_mapping2/types.h>
#include <hector_mapping2/internal/axes.h>
#include <hector_mapping2/internal/jet_opts.h>

#include <Eigen/Geometry>

namespace hector_mapping {

template <typename T, typename MatrixType>
void EulerAnglesToRotationMatrix(const T &roll, const T &pitch, const T &yaw, MatrixType &R)
{
  T sin_roll  = sin(roll);
  T cos_roll  = cos(roll);
  T sin_pitch = sin(pitch);
  T cos_pitch = cos(pitch);
  T sin_yaw   = sin(yaw);
  T cos_yaw   = cos(yaw);

//  R << cos_pitch*cos_yaw,                             cos_pitch*sin_yaw,                              -sin_pitch,
//       sin_roll*sin_pitch*cos_yaw - cos_roll*sin_yaw, sin_roll*sin_pitch*sin_yaw + cos_pitch*cos_yaw, cos_pitch*sin_roll,
//       cos_roll*sin_pitch*cos_yaw + sin_roll*sin_yaw, cos_roll*sin_pitch*sin_yaw - sin_roll*cos_yaw,  cos_pitch*cos_roll;
  R << cos_pitch*cos_yaw, sin_roll*sin_pitch*cos_yaw - cos_roll*sin_yaw,  cos_roll*sin_pitch*cos_yaw + sin_roll*sin_yaw,
       cos_pitch*sin_yaw, sin_roll*sin_pitch*sin_yaw + cos_pitch*cos_yaw, cos_roll*sin_pitch*sin_yaw - sin_roll*cos_yaw,
       -sin_pitch,        cos_pitch*sin_roll,                             cos_pitch*cos_roll;
}

//template <typename MapType, typename Axes>
//class OccupiedSpaceResidual {};

template <typename MapType>
// class OccupiedSpaceResidual<MapType, axes::XYZ> {
class OccupiedSpaceResidual {
public:
  OccupiedSpaceResidual(const MapType& map, const Point& endpoint, int level)
    : map_(map)
    , endpoint_(endpoint)
    , level_(level)
  {}

  template <typename T> bool operator()(const T* const xy, const T* const z, const T* const rollpitch, const T* const yaw, T* residual) const {
    Eigen::Matrix<T,3,1> translation(xy[0], xy[1], z[0]);
    Eigen::Matrix<T,3,3> rotation;
    EulerAnglesToRotationMatrix(rollpitch[0], rollpitch[1], yaw[0], rotation);

    Eigen::Matrix<T,4,4> transform = Eigen::Matrix<T,4,4>::Identity();
    transform.template block<3,3>(0,0) = rotation;
    transform.template block<3,1>(0,3) = translation;

    Eigen::Matrix<T,4,1> endpoint(T(endpoint_.x()), T(endpoint_.y()), T(endpoint_.z()), T(1.));
    Eigen::Matrix<T,4,1> world = transform * endpoint;
    T probability;
    if (map_.getValue(probability, world, level_))
      residual[0] = std::min(2. - 2. * probability, T(1.0));
    else
      residual[0] = T(0.);
    return true;
  }

private:
  const MapType& map_;
  const Point& endpoint_;
  const int level_;
};

//template <typename MapType, typename AxesType>
//class FreeSpaceResidual {};

template <typename MapType>
//class FreeSpaceResidual<MapType, axes::XY> {
class FreeSpaceResidual {
public:
  FreeSpaceResidual(const MapType& map, const Point& endpoint, int level, int steps)
    : map_(map)
    , endpoint_(endpoint)
    , level_(level)
    , steps_(steps)
  {}

  template <typename T> bool operator()(const T* const xy, const T* const z, const T* const rollpitch, const T* const yaw, T* residual) const {
    Eigen::Matrix<T,3,1> translation(xy[0], xy[1], z[0]);
    Eigen::Matrix<T,3,3> rotation;
    EulerAnglesToRotationMatrix(rollpitch[0], rollpitch[1], yaw[0], rotation);

    Eigen::Matrix<T,4,4> transform = Eigen::Matrix<T,4,4>::Identity();
    transform.template block<3,3>(0,0) = rotation;
    transform.template block<3,1>(0,3) = translation;

    float_t norm = endpoint_.template head<2>().norm();
    float_t voxel_diagonal_length = map_.getResolution(level_).template head<2>().norm();

    for (int i = 0; i < steps_; ++i) {
      float_t free_voxel_distance = norm - ((i + 1) * voxel_diagonal_length);
      if (free_voxel_distance < 0.) {
        residual[i] = T(0.);
      } else {
        float_t factor = free_voxel_distance / norm;
        Eigen::Matrix<T, 4, 1> point(T(factor * endpoint_.x()), T(factor * endpoint_.y()), T(factor * endpoint_.z()), T(1.));
        Eigen::Matrix<T, 4, 1> world = transform * point;
        T probability;
        if (map_.getValue(probability, world, level_))
          residual[i] = probability;
        else
          residual[i] = T(0.);
      }
    }
    return true;
  }

 private:
  const MapType& map_;
  const Point& endpoint_;
  const int level_;
  const int steps_;
};

//class MotionResidual {
// public:
//  MotionResidual(double x, double y, double theta)
//      : x_(x), y_(y), theta_(theta) {}

//  template <typename T> bool operator()(const T* const xy, const T* const z, const T* const rollpitch, const T* const yaw, T* residual) const {
//    Eigen::Matrix<T, 3, 1> initial_pose_estimate(
//        (T(x_)), (T(y_)), (T(theta_)));
//    Eigen::Matrix<T, 3, 1> current_pose_estimate(
//        x[0], y[0], theta[0]);
//    Eigen::Matrix<T, 3, 1> pose_estimate_delta =
//        current_pose_estimate - initial_pose_estimate;
//    residual[0] = pose_estimate_delta.squaredNorm();
//    return true;
//  }

// private:
//  const double x_;
//  const double y_;
//  const double theta_;
//};

}

#endif // HECTOR_MAPPING2_MATCHER_RESIDUALS_H
