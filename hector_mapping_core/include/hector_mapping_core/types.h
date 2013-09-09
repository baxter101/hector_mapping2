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


#ifndef HECTOR_MAPPING_TYPES_H
#define HECTOR_MAPPING_TYPES_H

#include <boost/array.hpp>
#include <boost/shared_ptr.hpp>

#include <Eigen/Core>
#include <geometry_msgs/TransformStamped.h>

namespace hector_mapping
{

  using std::size_t;
  typedef int index_t;
  typedef float float_t;
  typedef boost::array<index_t,3> GridIndex;

  typedef Eigen::Matrix<float_t,3,1> Point;
  typedef Eigen::Matrix<float_t,3,1> Resolution;
  typedef Eigen::Matrix<size_t,3,1> Size;

  class MapBase;
  class GridMapBase;
  class GridMapParameters;
  template <typename CellType> class GridMap;
  typedef boost::shared_ptr<GridMapBase> GridMapPtr;

  class GridCellBase;
  class OccupancyGridCell;

  class Scan;
  class ScanMatcher;
  typedef boost::shared_ptr<ScanMatcher> ScanMatcherPtr;

  using std_msgs::Header;
  using geometry_msgs::Transform;
  using geometry_msgs::TransformStamped;

  namespace internal {

    template <typename ParameterType> struct ParameterAdaptor : public ParameterType
    {
      template <typename OtherParameterType> ParameterAdaptor(const OtherParameterType& params)
      {
        const ParameterType *other = dynamic_cast<const ParameterType *>(&params);
        if (other) *this = *other;
      }
    };

  } // namespace internal

} // namespace hector_mapping

#endif // HECTOR_MAPPING_TYPES_H
