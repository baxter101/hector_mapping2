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


#ifndef HECTOR_MAPPING_STRUCTURE_AXIS_H
#define HECTOR_MAPPING_STRUCTURE_AXIS_H

#include <hector_mapping_core/types.h>
#include <hector_mapping_core/internal/macros.h>


namespace hector_mapping {
namespace structure {

// axis alignment variants
namespace axis {
  struct X  {
    static const size_t BinaryTreeNumberOfChildren = 2;
    static size_t getBinaryTreeIndex(const GridIndex& key, int bit) { return BIT(key[0], bit); }
    static void setBinaryTreeSize(Size& size, size_t dimension) { size = Size(dimension, 1, 1); }
    static size_t getArraySize(const Size& size) { return size.x(); }
    static size_t getArrayIndex(const GridIndex& key, const Size& size) { return static_cast<size_t>(key[0] + (size.x() >> 1)); }
  };
  struct Y  {
    static const size_t BinaryTreeNumberOfChildren = 2;
    static size_t getBinaryTreeIndex(const GridIndex& key, int bit) { return BIT(key[1], bit); }
    static void setBinaryTreeSize(Size& size, size_t dimension) { size = Size(1, dimension, 1); }
    static size_t getArraySize(const Size& size) { return size.y(); }
    static size_t getArrayIndex(const GridIndex& key, const Size& size) { return static_cast<size_t>(key[1] + (size.y() >> 1)); }
  };
  struct Z  {
    static const size_t BinaryTreeNumberOfChildren = 2;
    static size_t getBinaryTreeIndex(const GridIndex& key, int bit) { return BIT(key[2], bit); }
    static void setBinaryTreeSize(Size& size, size_t dimension) { size = Size(1, 1, dimension); }
    static size_t getArraySize(const Size& size) { return size.z(); }
    static size_t getArrayIndex(const GridIndex& key, const Size& size) { return static_cast<size_t>(key[2] + (size.z() >> 1)); }
  };
  struct XY {
    static const size_t BinaryTreeNumberOfChildren = 4;
    static size_t getBinaryTreeIndex(const GridIndex& key, int bit) { return (BIT(key[1], bit) << 1) | BIT(key[0], bit); }
    static void setBinaryTreeSize(Size& size, size_t dimension) { size = Size(dimension, dimension, 1); }
    static size_t getArraySize(const Size& size) { return size.x() * size.y(); }
    static size_t getArrayIndex(const GridIndex& key, const Size& size) { return static_cast<size_t>(key[1] + (size.y() >> 1)) * size.x() + static_cast<size_t>(key[0] + (size.x() >> 1)); }
  };
  struct YZ {
    static const size_t BinaryTreeNumberOfChildren = 4;
    static size_t getBinaryTreeIndex(const GridIndex& key, int bit) { return (BIT(key[2], bit) << 1) | BIT(key[1], bit); }
    static void setBinaryTreeSize(Size& size, size_t dimension) { size = Size(1, dimension, dimension); }
    static size_t getArraySize(const Size& size) { return size.y() * size.z(); }
    static size_t getArrayIndex(const GridIndex& key, const Size& size) { return static_cast<size_t>(key[2] + (size.z() >> 1)) * size.y() + static_cast<size_t>(key[1] + (size.y() >> 1)); }
  };
  struct XZ {
    static const size_t BinaryTreeNumberOfChildren = 4;
    static size_t getBinaryTreeIndex(const GridIndex& key, int bit) { return (BIT(key[2], bit) << 1) | BIT(key[0], bit); }
    static void setBinaryTreeSize(Size& size, size_t dimension) { size = Size(dimension, 1, dimension); }
    static size_t getArraySize(const Size& size) { return size.x() * size.z(); }
    static size_t getArrayIndex(const GridIndex& key, const Size& size) { return static_cast<size_t>(key[2] + (size.z() >> 1)) * size.x() + static_cast<size_t>(key[0] + (size.x() >> 1)); }
  };
  struct XYZ {
    static const size_t BinaryTreeNumberOfChildren = 8;
    static size_t getBinaryTreeIndex(const GridIndex& key, int bit) { return (BIT(key[2], bit) << 2) | (BIT(key[1], bit) << 1) | BIT(key[0], bit); }
    static void setBinaryTreeSize(Size& size, size_t dimension) { size = Size(dimension, dimension, dimension); }
    static size_t getArraySize(const Size& size) { return size.x() * size.y() * size.z(); }
    static size_t getArrayIndex(const GridIndex& key, const Size& size) { return static_cast<size_t>(key[2] + (size.z() >> 1)) * size.x() * size.y() + static_cast<size_t>(key[1] + (size.y() >> 1)) * size.x() + static_cast<size_t>(key[0] + (size.x() >> 1)); }
  };
}

} // namespace structure
} // namespace hector_mapping


#endif // HECTOR_MAPPING_STRUCTURE_AXIS_H
