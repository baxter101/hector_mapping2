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
#include <hector_mapping_core/map/occupancy.h>

#include <hector_mapping_core/structure/array.h>
#include <hector_mapping_core/structure/binary_tree.h>

#ifndef HECTOR_MAPPING_MAP_TYPES_H
#define HECTOR_MAPPING_MAP_TYPES_H

namespace hector_mapping {

  extern template class OccupancyGridMap<OccupancyGridCell>;

  extern template class structure::Array<OccupancyGridCell, axes::XY>;
  extern template class GridMapImpl<OccupancyGridMap<OccupancyGridCell>, structure::Array<OccupancyGridCell, axes::XY> >;
  typedef GridMapImpl<OccupancyGridMap<OccupancyGridCell>, structure::Array<OccupancyGridCell, axes::XY> > OccupancyGridMap2D;

  extern template class structure::BinaryTree<OccupancyGridCell, axes::XY>;
  extern template class GridMapImpl<OccupancyGridMap<OccupancyGridCell>, structure::BinaryTree<OccupancyGridCell, axes::XY> >;
  typedef GridMapImpl<OccupancyGridMap<OccupancyGridCell>, structure::BinaryTree<OccupancyGridCell, axes::XY> > OccupancyQuadTreeMap2D;

  extern template class structure::BinaryTree<OccupancyGridCell, axes::XYZ>;
  extern template class GridMapImpl<OccupancyGridMap<OccupancyGridCell>, structure::BinaryTree<OccupancyGridCell, axes::XYZ> >;
  typedef GridMapImpl<OccupancyGridMap<OccupancyGridCell>, structure::BinaryTree<OccupancyGridCell, axes::XYZ> > OccupancyOcTreeMap3D;

  extern template class structure::Array<OccupancyGridCell, axes::Z>;
  extern template class structure::BinaryTree<structure::Array<OccupancyGridCell, axes::Z>, axes::XY>;
  extern template class GridMapImpl<OccupancyGridMap<OccupancyGridCell>, structure::BinaryTree<structure::Array<OccupancyGridCell, axes::Z>, axes::XY> >;
  typedef GridMapImpl<OccupancyGridMap<OccupancyGridCell>, structure::BinaryTree<structure::Array<OccupancyGridCell, axes::Z>, axes::XY> > OccupancyQuadTreeMap3D;

} // namespace hector_mapping

#endif // HECTOR_MAPPING_MAP_TYPES_H
