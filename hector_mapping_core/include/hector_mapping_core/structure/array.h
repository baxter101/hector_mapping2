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

#include <hector_mapping_core/structure/axis.h>
#include <vector>

namespace hector_mapping {

// The ArrayParameters class.

class ArrayParameters
{
  PARAMETER(ArrayParameters, Size, size);

public:
  ArrayParameters()
    : size_(0,0,0)
  {}
};

namespace structure {

// The Array class.

template <typename T, typename Axis>
class Array
{
public:
  typedef Array<T, Axis> ThisType;
  typedef T NestedType;
  typedef ArrayParameters Parameters;

  template <typename ParameterType> Array(const ParameterType& params = ParameterType())
  {
    internal::ParameterAdaptor<Parameters> p(params);
    resize(p.size());
  }

  virtual ~Array() {}

  const Size &getSize() const { return size_; }

  virtual T *get(const GridIndex& key) {
    return &(array_[Axis::getArrayIndex(key, size_)]);
  }

  virtual const T *get(const GridIndex& key) const {
    return &(array_[Axis::getArrayIndex(key, size_)]);
  }

  virtual void resize(const Size& size) {
    std::size_t new_size = Axis::getArraySize(size);
    if (array_.size() != new_size) {
      array_.resize(new_size);
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
