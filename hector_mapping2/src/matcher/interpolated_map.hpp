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

#ifndef HECTOR_MAPPING2_MATCHER_INTERPOLATED_MAP_H
#define HECTOR_MAPPING2_MATCHER_INTERPOLATED_MAP_H

#include <hector_mapping2/map.h>
#include <hector_mapping2/internal/axes.h>
#include <hector_mapping2/internal/jet_opts.h>

namespace hector_mapping {

template <typename MapType, typename Axes>
class LinearInterpolatedMap {};

template <typename MapType>
class LinearInterpolatedMap<MapType, axes::XY> {
public:
  LinearInterpolatedMap(const MapType& map) : map_(map) {}

  const Resolution& getResolution(int level) const { return map_.getResolution(level); }

  template <typename T, typename Derived> bool getValue(T& value, const Eigen::MatrixBase<Derived> &point, int level) const {
    Resolution resolution = getResolution(level);

    // lower-left center
    Point p0((std::floor(ceres::JetOps<T>::GetScalar(point.x()) / resolution.x() - 0.5f) + 0.5f) * resolution.x(),
             (std::floor(ceres::JetOps<T>::GetScalar(point.y()) / resolution.y() - 0.5f) + 0.5f) * resolution.y(),
             (std::floor(ceres::JetOps<T>::GetScalar(point.z()) / resolution.z() - 0.5f) + 0.5f) * resolution.z());
    // upper-right center
    Point p1(p0 + resolution);

    Eigen::Matrix<typename MapType::ValueType,4,1> P;
    if (!getInterpolationMatrix(map_, map_.toGridIndex(p0), level, P)) return false;

    value = ((point.y() - T(p0.y())) * ((point.x() - T(p0.x())) * T(P(3)) + (T(p1.x()) - point.x()) * T(P(1))) +
             (T(p1.y()) - point.y()) * ((point.x() - T(p0.x())) * T(P(2)) + (T(p1.x()) - point.x()) * T(P(0)))) / T((p1.y() - p0.y()) * (p1.x() - p0.x()));
    return true;
  }

private:
  template <typename MatrixType>
  static bool getInterpolationMatrix(const MapType& map, const GridIndex& lower_left, int level, MatrixType& P) {
    index_t step = 1u << level;
    for (index_t dx = 0; dx < 2; ++dx) {
      for (index_t dy = 0; dy < 2; ++dy) {
        typename MapType::ValueType value = map.getValue(GridIndex(lower_left.x() + dx * step, lower_left.y() + dy * step, lower_left.z()), level);
        if (!(value == value)) return false;
        P(dx << 1 | dy) = (typename MatrixType::Scalar)(value);
      }
    }
    return true;
  }

private:
  const MapType& map_;
};

template <typename MapType>
class LinearInterpolatedMap<MapType, axes::XYZ> {
public:
  LinearInterpolatedMap(const MapType& map) : map_(map) {}

  const Resolution& getResolution(int level) const { return map_.getResolution(level); }

  template <typename T, typename Derived> bool getValue(T& value, const Eigen::MatrixBase<Derived> &point, int level) const {
    Resolution resolution = getResolution(level);

    // lower-left center
    Point p0((std::floor(ceres::JetOps<T>::GetScalar(point.x()) / resolution.x() - 0.5f) + 0.5f) * resolution.x(),
             (std::floor(ceres::JetOps<T>::GetScalar(point.y()) / resolution.y() - 0.5f) + 0.5f) * resolution.y(),
             (std::floor(ceres::JetOps<T>::GetScalar(point.z()) / resolution.z() - 0.5f) + 0.5f) * resolution.z());
    // upper-right center
    Point p1(p0 + resolution);

    Eigen::Matrix<typename MapType::ValueType,8,1> P;
    if (!getInterpolationMatrix(map_, map_.toGridIndex(p0), level, P)) return false;

    value = ((point.z() - T(p0.z())) *
               ((point.y() - T(p0.y())) *
                  ((point.x() - T(p0.x())) * T(P(7)) + (T(p1.x()) - point.x()) * T(P(5))) +
                (T(p1.y()) - point.y()) *
                  ((point.x() - T(p0.x())) * T(P(6)) + (T(p1.x()) - point.x()) * T(P(4)))
               )
           + (T(p1.z()) - point.z()) *
               ((point.y() - T(p0.y())) *
                  ((point.x() - T(p0.x())) * T(P(3)) + (T(p1.x()) - point.x()) * T(P(1))) +
                (T(p1.y()) - point.y()) *
                  ((point.x() - T(p0.x())) * T(P(2)) + (T(p1.x()) - point.x()) * T(P(0)))
               )
            ) / T((p1.z() - p0.z()) * (p1.y() - p0.y()) * (p1.x() - p0.x()));
    return true;
  }

private:
  template <typename MatrixType>
  static bool getInterpolationMatrix(const MapType& map, const GridIndex& lower_left, int level, MatrixType& P) {
    index_t step = 1u << level;
    for (index_t dx = 0; dx < 2; ++dx) {
      for (index_t dy = 0; dy < 2; ++dy) {
        for (index_t dz = 0; dz < 2; ++dz) {
          typename MapType::ValueType value = map.getValue(GridIndex(lower_left.x() + dx * step, lower_left.y() + dy * step, lower_left.z() + dz * step), level);
          if (!(value == value)) return false;
          P(dx << 2 | dy << 1 | dz) = (typename MatrixType::Scalar)(value);
        }
      }
    }
    return true;
  }

private:
  const MapType& map_;
};

} // namespace hector_mapping

#endif // HECTOR_MAPPING2_MATCHER_INTERPOLATED_MAP_H
