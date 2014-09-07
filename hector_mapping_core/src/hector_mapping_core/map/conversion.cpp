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

#include <hector_mapping_core/map/conversion.h>
#include <ros/console.h>

namespace hector_mapping
{

void toOccupancyGridMessage(const OccupancyGridMapBase& map, nav_msgs::OccupancyGrid& message, float_t z) {

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

  Point min_point, max_point;
  map.getExtends(min_point, max_point);
  message.info.origin.position.x = min_point.x();
  message.info.origin.position.y = min_point.y();
  message.info.origin.position.z = min_point.z();
  message.info.origin.orientation.w = 1.0;
  message.info.origin.orientation.x = message.info.origin.orientation.y = message.info.origin.orientation.z = 0;

  // write map
  message.data.resize(message.info.width * message.info.height);
  int8_t *data = message.data.data();
  GridIndex min_index, max_index;
  map.getExtends(min_index, max_index);
  GridIndex index(0, 0, map.toGridIndexAxis(z, 2));
  for(index[1] = min_index[1]; index[1] <= max_index[1]; index[1]++) {
    for(index[0] = min_index[0]; index[0] <= max_index[0]; index[0]++, data++) {
      const OccupancyGridCell* cell = map.getOccupancy(index, level::SEARCH);
      if (cell->isFree(map.getOccupancyParameters())) *data = 0;
      else if (cell->isOccupied(map.getOccupancyParameters())) *data = 100;
      else *data = -1;
    }
  }
}

} // namespace hector_mapping
