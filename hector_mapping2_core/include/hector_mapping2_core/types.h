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


#ifndef HECTOR_MAPPING2_TYPES_H
#define HECTOR_MAPPING2_TYPES_H

#include <boost/array.hpp>
#include <boost/shared_ptr.hpp>

#include <boost/functional/hash.hpp>
#include <boost/unordered_map.hpp>
#include <boost/unordered_set.hpp>

#include <Eigen/Geometry>
#include <tf/transform_datatypes.h>

#ifdef __GNUC__
  #include <tr1/unordered_set>
  #include <tr1/unordered_map>
#else
  #include <unordered_set>
  #include <unordered_map>
#endif

namespace hector_mapping
{
  template <typename T, std::size_t N>
  struct array : public boost::array<T,N>
  {
    using boost::array<T,N>::operator[];
    array() {}
    array(const T &x, const T &y)             { (*this)[0] = x; (*this)[1] = y; }
    array(const T &x, const T &y, const T &z) { (*this)[0] = x; (*this)[1] = y; (*this)[2] = z; }
    T &x()             { return (*this)[0]; }
    const T &x() const { return (*this)[0]; }
    T &y()             { return (*this)[1]; }
    const T &y() const { return (*this)[1]; }
    T &z()             { return (*this)[2]; }
    const T &z() const { return (*this)[2]; }
    static array<T,N> Zero() { array<T,N> zero; zero.assign(T()); return zero; }
  };

  using std::size_t;
  typedef short int index_t;
  typedef short int diff_t;
  typedef float float_t;
  typedef array<index_t,3> GridIndex;
  typedef array<size_t,3> Size;

  typedef Eigen::Matrix<float_t,2,1> Point2;
  typedef Eigen::Matrix<float_t,3,1> Point3;
  typedef Eigen::Matrix<float_t,3,1> Point;
  typedef Eigen::Matrix<float_t,3,1> Resolution;

  class MapFactory;
  class MapBase;
  typedef boost::shared_ptr<MapBase> MapBasePtr;
  class GridMapBase;
  typedef boost::shared_ptr<GridMapBase> GridMapPtr;
  template <typename CellType, typename BaseType> class GridMap;

  class OccupancyGridMapBase;
  typedef boost::shared_ptr<OccupancyGridMapBase> OccupancyGridMapPtr;
  template <typename OccupancyCellType> class OccupancyGridMap;

  class GridCellBase;
  class OccupancyGridCell;

  class Scan;
  class ScanMatcher;
  typedef boost::shared_ptr<ScanMatcher> ScanMatcherPtr;

  // levels
  namespace level {
    enum { MAX = -1, SEARCH = -2 };
  }

  // stream operator for GridIndex
  static inline std::ostream& operator<<(std::ostream& os, const GridIndex& index) {
    os << "[" << index[0] << "," << index[1] << "," << index[2] << "]";
    return os;
  }
} // namespace hector_mapping

namespace boost {
  template <> struct hash<hector_mapping::GridIndex> {
    size_t operator()(const hector_mapping::GridIndex& key) const{
      return key[0] + 1337*key[1] + 345637*key[2];
    }
  };
}

namespace hector_mapping {
  typedef std::pair<GridIndex,int> CacheKey;
  typedef boost::unordered_set<GridIndex> GridIndexSet;
  typedef boost::unordered_map<GridIndex, bool> GridIndexMap;
} // namespace hector_mapping

#include <hector_mapping2_core/internal/comparisons.h>

#endif // HECTOR_MAPPING2_TYPES_H
