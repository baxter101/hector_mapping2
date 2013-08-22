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


#ifndef HECTOR_MAPPING_MAP_H
#define HECTOR_MAPPING_MAP_H

#include <hector_mapping_core/forwards.h>

#include <boost/array.hpp>
#include <Eigen/Geometry>

#include <vector>

namespace hector_mapping
{

typedef int index_t;
typedef float float_t;
typedef boost::array<index_t,3> GridIndex;

typedef Eigen::Matrix<float_t,3,1> Point;
typedef Eigen::Matrix<float_t,3,1> Offset;
typedef Eigen::Matrix<float_t,3,1> Scale;
typedef Eigen::Matrix<index_t,3,1> Size;

class MapBase
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

public:
  virtual ~MapBase() {}

  virtual void getExtends(Point &min, Point &max) = 0;
  virtual void setExtends(const Point &min, const Point &max) = 0;
  virtual void setMinExtends(const Point &point) = 0;

  const Offset& getOffset() const { return offset_; }
  virtual void setOffset(const Point &offset) { offset_ = offset; }

  virtual void clear() = 0;

protected:
  Point offset_;
};

class GridMapBase : public MapBase
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

public:
  virtual ~GridMapBase() {}

  virtual void getExtends(Point &min, Point &max);
  virtual void setExtends(const Point &min, const Point &max);
  virtual void growMinExtends(const Point &point);

  const Scale& getScale() const { return scale_; }
  virtual void setScale(const Scale& scale) = 0;

  const Size& getSize() const { return size_; }
  virtual void setSize(const Size& size) = 0;

  virtual GridIndex toGridIndex(const Point &point) = 0;
  virtual Point toPoint(const GridIndex &key) = 0;

  virtual GridIndex getNeighbourIndex(const GridIndex &key, unsigned int axis, unsigned int step = 1) = 0;

protected:
  Scale scale_;
  Size size_;
};

class GridMapParameters
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

public:
  GridMapParameters();
  GridMapParameters &size(index_t size);
  GridMapParameters &size(const Size& size);
  GridMapParameters &scale(float_t scale);
  GridMapParameters &scale(const Scale &scale);
  GridMapParameters &offset(const Point &offset);

  Size size_;
  Point offset_;
  Scale scale_;
};

template <typename CellType>
class GridMap : public GridMapBase
{
public:
  virtual ~GridMap() {}

  virtual CellType& get(const GridIndex& key) = 0;
  virtual const CellType& get(const GridIndex& key) const = 0;

  // convenience access functions
  CellType& operator()(const GridIndex& key)             { return get(key); }
  const CellType& operator()(const GridIndex& key) const { return get(key); }
  CellType& operator()(const Point& point)               { return get(toGridIndex(point)); }
  const CellType& operator()(const Point& point) const   { return get(toGridIndex(point)); }
};

} // namespace hector_mapping

#endif // HECTOR_MAPPING_MAP_H
