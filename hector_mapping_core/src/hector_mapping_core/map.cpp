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

#include <hector_mapping_core/map.h>
#include <hector_mapping_core/internal/macros.h>

#include <hector_mapping_core/map/conversion.h>

#include <assert.h>
#include <ros/console.h>

namespace hector_mapping
{

void GridMapBase::getExtends(Point &min, Point &max)
{
  min.x() = -(getSize().x() * resolution_.x() / 2.0) + offset_.x();
  min.y() = -(getSize().y() * resolution_.y() / 2.0) + offset_.y();
  min.z() = -(getSize().z() * resolution_.z() / 2.0) + offset_.z();
  max.x() =  (getSize().x() * resolution_.x() / 2.0) + offset_.x();
  max.x() =  (getSize().y() * resolution_.y() / 2.0) + offset_.y();
  max.x() =  (getSize().z() * resolution_.z() / 2.0) + offset_.z();
}

bool GridMapBase::setExtends(const Point &min, const Point &max)
{
  return false;
}

bool GridMapBase::growMinExtends(const Point &point)
{
  Point min, max;
  getExtends(min, max);
  if (point.x() < min.x()) min.x() = point.x();
  if (point.y() < min.y()) min.y() = point.y();
  if (point.z() < min.z()) min.z() = point.z();
  if (point.x() > max.x()) max.x() = point.x();
  if (point.y() > max.y()) max.y() = point.y();
  if (point.z() > max.z()) max.z() = point.z();
  return setExtends(min, max);
}

inline index_t GridMapBase::toGridIndexAxis(float_t coordinates, int axis) const
{
  return static_cast<index_t>(floor(((coordinates - offset_[axis]) / resolution_[axis]) + .5));
}

inline float_t GridMapBase::toPointAxis(index_t index, int axis) const
{
  return (static_cast<float_t>(index) + .5) * resolution_[axis] + offset_[axis];
}

inline GridIndex GridMapBase::toGridIndex(const Point &point) const
{
  GridIndex index;
  index[0] = toGridIndexAxis(point.x(), 0);
  index[1] = toGridIndexAxis(point.y(), 1);
  index[2] = toGridIndexAxis(point.z(), 2);
  return index;
}

inline Point GridMapBase::toPoint(const GridIndex &index) const
{
  Point point;
  point.x() = toPointAxis(index[0], 0);
  point.y() = toPointAxis(index[1], 1);
  point.z() = toPointAxis(index[2], 2);
  return point;
}

GridIndex GridMapBase::getNeighbourIndex(const GridIndex &key, unsigned int axis, int step) const
{
  GridIndex neighbor = key;
  neighbor[axis] += step;
  assert(neighbor[axis] >= getSize()[axis] / 2 && neighbor[axis] < getSize()[axis] / 2);
  return neighbor;
}

void toOccupancyGridMessage(const GridMapBase& map_in, nav_msgs::OccupancyGrid& message, float_t z) {
  // cast map in an OccupancyGridMap. This will throw if the GridMapBase is not an OccupancyGridMap.
  const OccupancyGridMap& map = dynamic_cast<const OccupancyGridMap &>(map_in);

  // set header
  message.header = map.getHeader();

  // set meta data
  if (map.getResolution().x() != map.getResolution().y()) {
    ROS_ERROR("GridMap cannot be converted to an OccupancyGrid message as x and y resolution differs!");
    message = nav_msgs::OccupancyGrid();
    return;
  }
  message.info.resolution = map.getResolution().x();
  message.info.width = map.getSize().x();
  message.info.height = map.getSize().y();
  message.info.origin.position.x = map.getOffset().x();
  message.info.origin.position.y = map.getOffset().y();
  message.info.origin.position.z = map.getOffset().z();
  message.info.origin.orientation.w = 1.0;
  message.info.origin.orientation.x = message.info.origin.orientation.y = message.info.origin.orientation.z = 0;

  // write map
  message.data.resize(message.info.width * message.info.height);
  int8_t *data = message.data.data();
  GridIndex index = { 0, 0, map.toGridIndexAxis(z, 2) };
  for( ; index[1] < map.getSize().y(); index[1]++) {
    for( ; index[0] < map.getSize().x(); index[0]++, data++) {
      const OccupancyGridCell& cell = map(index);
      if (cell.isFree(map.getCellParameters())) *data = 0;
      else if (cell.isOccupied(map.getCellParameters())) *data = 100;
      else *data = -1;
    }
  }
}

} // namespace hector_mapping

