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


#ifndef HECTOR_MAPPING_MATCHING_RESIDUALS_H
#define HECTOR_MAPPING_MATCHING_RESIDUALS_H

#include <hector_mapping_core/types.h>
#include <hector_mapping_core/internal/axes.h>
#include <hector_mapping_core/internal/jet_opts.h>

#include <Eigen/Geometry>

namespace hector_mapping {

template <typename MapType, typename Axes>
class OccupiedSpaceResidual {};

template <typename MapType>
class OccupiedSpaceResidual<MapType, axes::XY> {
public:
  OccupiedSpaceResidual(const MapType& map, const Point& endpoint, int level)
    : map_(map)
    , endpoint_(endpoint)
    , level_(level)
  {}

  template <typename T> bool operator()(const T* const x, const T* const y, const T* const theta, T* residual) const {
    Eigen::Matrix<T, 3, 1> translation(x[0], y[0], T(0.));
    Eigen::AngleAxis<T> rotation(theta[0], Eigen::Matrix<T,3,1>::UnitZ());
    Eigen::Matrix<T, 4, 4> transform;
    transform << rotation.toRotationMatrix(), translation, T(0.), T(0.), T(0.), T(1.);
//    std::cout << ceres::JetOps<T>::GetScalar(*x) << " " << ceres::JetOps<T>::GetScalar(*y) << " " << ceres::JetOps<T>::GetScalar(*theta) << ": " << std::endl;
//    Eigen::Matrix<double,4,4> transformd;
//    transformd << ceres::JetOps<T>::GetScalar(transform(0,0)), ceres::JetOps<T>::GetScalar(transform(0,1)), ceres::JetOps<T>::GetScalar(transform(0,2)), ceres::JetOps<T>::GetScalar(transform(0,3)),
//                  ceres::JetOps<T>::GetScalar(transform(1,0)), ceres::JetOps<T>::GetScalar(transform(1,1)), ceres::JetOps<T>::GetScalar(transform(1,2)), ceres::JetOps<T>::GetScalar(transform(1,3)),
//                  ceres::JetOps<T>::GetScalar(transform(2,0)), ceres::JetOps<T>::GetScalar(transform(2,1)), ceres::JetOps<T>::GetScalar(transform(2,2)), ceres::JetOps<T>::GetScalar(transform(2,3)),
//                  ceres::JetOps<T>::GetScalar(transform(3,0)), ceres::JetOps<T>::GetScalar(transform(3,1)), ceres::JetOps<T>::GetScalar(transform(3,2)), ceres::JetOps<T>::GetScalar(transform(3,3));
//    std::cout << transformd << std::endl;
    Eigen::Matrix<T, 4, 1> endpoint(T(endpoint_.x()), T(endpoint_.y()), T(endpoint_.z()), T(1.));
    Eigen::Matrix<T, 4, 1> world = transform * endpoint;
    T probability;
    if (map_.getValue(probability, world, level_))
      residual[0] = 1. - probability;
    else
      residual[0] = T(0.);
    return true;
  }

 private:
  const MapType& map_;
  const Point& endpoint_;
  const int level_;
};


template <typename MapType, typename AxesType>
class FreeSpaceResidual {};

template <typename MapType>
class FreeSpaceResidual<MapType, axes::XY> {
public:
  FreeSpaceResidual(const MapType& map, const Point& endpoint, int level, int steps)
    : map_(map)
    , endpoint_(endpoint)
    , level_(level)
    , steps_(steps)
  {}

  template <typename T> bool operator()(const T* const x, const T* const y, const T* const theta, T* residual) const {
    Eigen::Matrix<T, 3, 1> translation(x[0], y[0], T(0.));
    Eigen::AngleAxis<T> rotation(theta[0], Eigen::Matrix<T,3,1>::UnitZ());
    Eigen::Matrix<T, 4, 4> transform;
    transform << rotation.toRotationMatrix(), translation, T(0.), T(0.), T(0.), T(1.);

    float_t norm = endpoint_.norm();
    float_t voxel_diagonal_length = map_.getResolution(level_).norm();

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
          residual[0] = probability;
        else
          residual[0] = T(0.);
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

//  template <typename T> bool operator()(const T* const x, const T* const y, const T* const theta, T* residual) const {
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

#endif // HECTOR_MAPPING_MATCHING_RESIDUALS_H
