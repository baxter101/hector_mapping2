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

#include <hector_mapping2_core/map/types.h>

namespace hector_mapping {

  template class OccupancyGridMap<OccupancyGridCell>;

  // OccupancyGridMap2D
  template class structure::Array<OccupancyGridCell, axes::XY>;
  template class GridMapImpl<OccupancyGridMap<OccupancyGridCell>, structure::Array<OccupancyGridCell, axes::XY> >;

  // OccupancyQuadTreeMap2D
  template class structure::BinaryTree<OccupancyGridCell, axes::XY>;
  template class GridMapImpl<OccupancyGridMap<OccupancyGridCell>, structure::BinaryTree<OccupancyGridCell, axes::XY> >;

  // OccupancyOcTreeMap3D
  template class structure::BinaryTree<OccupancyGridCell, axes::XYZ>;
  template class GridMapImpl<OccupancyGridMap<OccupancyGridCell>, structure::BinaryTree<OccupancyGridCell, axes::XYZ> >;

  // OccupancyQuadTreeMap3D
  template class structure::Array<OccupancyGridCell, axes::Z>;
  template class structure::BinaryTree<structure::Array<OccupancyGridCell, axes::Z>, axes::XY>;
  template class GridMapImpl<OccupancyGridMap<OccupancyGridCell>, structure::BinaryTree<structure::Array<OccupancyGridCell, axes::Z>, axes::XY> >;

  // Factory
  MapBasePtr MapFactory::operator()(const std::string& type) {
    MapBasePtr map;
    if (type == "OccupancyGridMap2D") {
      map.reset(new OccupancyGridMap2D(params_));
    } else if (type == "OccupancyQuadTreeMap2D") {
      map.reset(new OccupancyQuadTreeMap2D(params_));
    } else if (type == "OccupancyOcTreeMap3D") {
      map.reset(new OccupancyOcTreeMap3D(params_));
    } else if (type == "OccupancyQuadTreeMap3D") {
      map.reset(new OccupancyQuadTreeMap3D(params_));
    }
    return map;
  }
}
