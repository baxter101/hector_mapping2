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


#ifndef HECTOR_MAPPING_STRUCTURE_ARRAY_H
#define HECTOR_MAPPING_STRUCTURE_ARRAY_H

#include <hector_mapping_core/structure/structure.h>
#include <hector_mapping_core/internal/axes.h>

#include <vector>

#include <ros/console.h>

namespace hector_mapping {
namespace structure {

// The Array class.

template <typename T, typename Axes>
class Array : public StructureBase
{
public:
  typedef Array<T, Axes> ThisType;
  typedef T NestedType;

  Array(const Parameters& params = Parameters())
    : StructureBase(params)
    , size_(Size::Zero())
  {
    params("map_size", size_);
    resize(size_);
  }
  virtual ~Array() {}

  const Size &getSize() const { return size_; }

  bool setExtends(const GridIndex &min, const GridIndex &max) {
    // todo: resize grid dynamically
    return false;
  }

  T *get(const GridIndex& key, int) {
    return &(array_[Axes::getArrayIndex(key, size_)]);
  }

  const T *get(const GridIndex& key, int) const {
    return &(array_[Axes::getArrayIndex(key, size_)]);
  }

  virtual void resize(Size size) {
    Axes::adjustSize(size);
    std::size_t array_size = Axes::getArraySize(size);
    if (array_.size() != array_size) {
      array_.resize(array_size);
      clear();
    }
    size_ = size;
  }

  virtual void clear() {
    array_.assign(array_.size(), T());
  }

protected:
  Size size_;
  std::vector<T> array_;
};

} // namespace structure
} // namespace hector_mapping

#endif // HECTOR_MAPPING_STRUCTURE_ARRAY_H
