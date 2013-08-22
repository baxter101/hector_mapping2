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


#ifndef HECTOR_MAPPING_MAP_QUADTREE_H
#define HECTOR_MAPPING_MAP_QUADTREE_H

#include <hector_mapping_core/map.h>
#include <boost/shared_array.hpp>

namespace hector_mapping
{

template <typename T>
struct QuadTreeNode
{
  QuadTreeNode() : data_(0), children_(0) {}
  ~QuadTreeNode() { delete[] data_; delete[] children_; }

  void clear() {
    delete[] data_; data_ = 0;
    delete[] children_; children_ = 0;
  }

  T *data_;
  QuadTreeNode<T> *children_;
};

template <typename CellType>
struct Flat
{
  CellType& get(const GridIndex& key) { return data_; }
  const CellType& get(const GridIndex& key) const { return data_; }

private:
  CellType data_;
};

class QuadTreeMapParameters : public GridMapParameters {
  QuadTreeMapParameters()
    : GridMapParameters()
    , max_depth(16)
  {}

  QuadTreeMapParameters &max_depth(int max_depth);
  int max_depth_;
};

template <typename CellType, typename ZAxisRepresentation = Flat<CellType> >
class QuadTreeMap : public GridMap<CellType>
{
public:
  typedef QuadTreeNode<ZAxisRepresentation> Node;

  QuadTreeMap(const QuadTreeMapParameters& params = QuadTreeMapParameters());
  virtual ~QuadTreeMap();

  virtual T& get(const GridIndex& key, int max_depth = -1);
  virtual const T& get(const GridIndex& key, int max_depth = -1) const;

  virtual void clear() {
    delete root_;
    root_ = 0;
  }

private:
  Node *root_;
  int max_depth_;
};

} // namespace hector_mapping

#endif // HECTOR_MAPPING_MAP_QUADTREE_H
