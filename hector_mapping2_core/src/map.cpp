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

#include <hector_mapping2_core/map.h>
#include <assert.h>

namespace hector_mapping
{

MapFactory::MapFactory(const Parameters &_params) : params_(_params) {}

MapBase::MapBase(const Parameters &_params)
  : params_(_params)
  , empty_(true)
{
  params()("map_offset", offset_).default_value(Point(0.0, 0.0, 0.0));
  header_.frame_id = params().get<std::string>("map_frame");
}

void MapBase::reset()
{
  empty_ = true;
}

void MapBase::updated()
{
  empty_ = false;
}

GridMapBase::GridMapBase(const Parameters& _params)
  : MapBase(_params)
{
  params()("map_resolution", resolution_).default_value(Resolution(0.05, 0.05, 0.05));
}

void GridMapBase::getExtends(Point &min, Point &max) const
{
  GridIndex min_index, max_index;
  getExtends(min_index, max_index);
  min = toPoint(min_index) - getResolution() / 2.0;
  max = toPoint(max_index) + getResolution() / 2.0;
  if (getSize().x() == 0) { min.x() = -std::numeric_limits<float_t>::infinity(); max.x() = std::numeric_limits<float_t>::infinity(); }
  if (getSize().y() == 0) { min.y() = -std::numeric_limits<float_t>::infinity(); max.y() = std::numeric_limits<float_t>::infinity(); }
  if (getSize().z() == 0) { min.z() = -std::numeric_limits<float_t>::infinity(); max.z() = std::numeric_limits<float_t>::infinity(); }
}

void GridMapBase::getExtends(GridIndex &min, GridIndex &max) const
{
  min.x() = -(getSize().x() / 2);
  min.y() = -(getSize().y() / 2);
  min.z() = -(getSize().z() / 2);
  max.x() =  (getSize().x() - 1) / 2;
  max.y() =  (getSize().y() - 1) / 2;
  max.z() =  (getSize().z() - 1) / 2;
  if (getSize().x() == 0) { min.x() = -std::numeric_limits<index_t>::max(); max.x() = std::numeric_limits<index_t>::max(); }
  if (getSize().y() == 0) { min.y() = -std::numeric_limits<index_t>::max(); max.y() = std::numeric_limits<index_t>::max(); }
  if (getSize().z() == 0) { min.z() = -std::numeric_limits<index_t>::max(); max.z() = std::numeric_limits<index_t>::max(); }
}

bool GridMapBase::isValid(const GridIndex &index) const
{
  GridIndex current_min, current_max;
  getExtends(current_min, current_max);
  return hector_mapping::isValid(index, current_min, current_max);
}

bool GridMapBase::setExtends(const Point &min, const Point &max)
{
  return setExtends(toGridIndex(min), toGridIndex(max));
}

bool MapBase::growMinExtends(const Point &point)
{
  Point min, max;
  getExtends(min, max);
  return setExtends(minPoint(min, point),  maxPoint(max, point));
}

bool GridMapBase::growMinExtends(const GridIndex &index)
{
  GridIndex min, max;
  getExtends(min, max);

  GridIndex new_min = minGridIndex(min, index);
  GridIndex new_max = maxGridIndex(max, index);
  if (new_min == min && new_max == max) return true;
  return setExtends(new_min, new_max);
}

inline index_t GridMapBase::toGridIndexAxis(float_t coordinates, int axis) const
{
//  return static_cast<index_t>(floor(((coordinates - offset_.get()[axis]) / resolution_.get()[axis]) + .5f));
  return static_cast<index_t>(floor((coordinates - offset_.get()[axis]) / resolution_.get()[axis]));
}

inline float_t GridMapBase::toPointAxis(index_t index, int axis) const
{
//  return (static_cast<float_t>(index) * resolution_.get()[axis]) + offset_.get()[axis];
  return ((static_cast<float_t>(index) + .5f) * resolution_.get()[axis]) + offset_.get()[axis];
}

inline GridIndex GridMapBase::toGridIndex(const Point &point) const
{
  GridIndex index;
  index.x() = toGridIndexAxis(point.x(), 0);
  index.y() = toGridIndexAxis(point.y(), 1);
  index.z() = toGridIndexAxis(point.z(), 2);
  return index;
}

inline Point GridMapBase::toPoint(const GridIndex &index) const
{
  Point point;
  point.x() = toPointAxis(index.x(), 0);
  point.y() = toPointAxis(index.y(), 1);
  point.z() = toPointAxis(index.z(), 2);
  return point;
}

GridIndex GridMapBase::getNeighbourIndex(const GridIndex &key, unsigned int axis, int step) const
{
  GridIndex neighbor = key;
  neighbor[axis] += step;
  assert(neighbor[axis] >= getSize()[axis] / 2 && neighbor[axis] < getSize()[axis] / 2);
  return neighbor;
}

} // namespace hector_mapping

