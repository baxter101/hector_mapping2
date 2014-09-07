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


#ifndef HECTOR_MAPPING_STRUCTURE_GET_CELL_H
#define HECTOR_MAPPING_STRUCTURE_GET_CELL_H

#include <boost/type_traits/is_same.hpp>
#include <boost/utility/enable_if.hpp>

namespace hector_mapping {

template <typename CellType, typename Structure, class enabled = void>
struct GetCell
{
  static CellType *get(Structure& structure, const GridIndex& index, int level = 0) {
    typename Structure::NestedType *nested = structure.get(index, level);
    if (!nested) return 0;
    return GetCell<CellType, typename Structure::NestedType>::get(*nested, index, level);
  }

  static const CellType *get(const Structure& structure, const GridIndex& index, int level = 0) {
    typename Structure::NestedType const *nested = structure.get(index, level);
    if (!nested) return 0;
    return GetCell<CellType, typename Structure::NestedType>::get(*nested, index, level);
  }
};

template <typename CellType, typename Structure>
struct GetCell<CellType, Structure, typename boost::enable_if< typename boost::is_same<CellType, typename Structure::NestedType>::type >::type >
{
  static CellType *get(Structure& structure, const GridIndex& index, int level = 0)             { return structure.get(index, level); }
  static const CellType *get(const Structure& structure, const GridIndex& index, int level = 0) { return structure.get(index, level); }
};

} // namespace hector_mapping

#endif // HECTOR_MAPPING_STRUCTURE_GET_CELL_H
