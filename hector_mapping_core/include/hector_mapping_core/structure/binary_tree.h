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


#ifndef HECTOR_MAPPING_STRUCTURE_BINARY_TREE_H
#define HECTOR_MAPPING_STRUCTURE_BINARY_TREE_H

#include <hector_mapping_core/structure/axis.h>
#include <boost/array.hpp>

namespace hector_mapping {
namespace structure {

// The BinaryTreeNode class.
template <typename T, std::size_t BinaryTreeNumberOfChildren>
struct BinaryTreeNode
{
public:
  typedef BinaryTreeNode<T,BinaryTreeNumberOfChildren> ThisType;
  typedef T NestedType;

  typedef ThisType * Ptr;
  typedef ThisType const * ConstPtr;
  typedef boost::array<Ptr,BinaryTreeNumberOfChildren> Children;
  typedef typename Children::size_type size_type;

  BinaryTreeNode() : data_(), children_(0) {}
  ~BinaryTreeNode() { clear(); }

  void clear() {
    if (children_) {
      for(typename Children::const_iterator it = children_->begin(); it != children_->end(); ++it) delete *it;
      delete children_; children_ = 0;
    }
  }

  void expand(const size_type i) {
    if (!children_) {
      children_ = new Children;
      children_->assign(0);
    }
    children_->at(i) = new ThisType();
  }

  void prune(const size_type i) {
    static Children all_zero = {{ 0, 0, 0, 0, 0, 0, 0, 0 }};
    if (children_) {
      delete children_->at(i);
      children_->at(i) = 0;
      if (*children_ == all_zero) {
        delete children_;
        children_ = 0;
      }
    }
  }

  bool hasChildren() const { return children_; }

  Ptr get(const size_type i) { expand(i); return children_->at(i); }
  ConstPtr get(const size_type i) const { return children_ ? children_->at(i) : 0; }

  T* data() { return &data_; }
  const T* data() const { return &data_; }

  T& operator*() { return data_; }
  const T& operator*() const { return data_; }

private:
  T data_;
  Children *children_;
};

// The BinaryTreeParameters class.

class BinaryTreeParameters {
public:
  BinaryTreeParameters()
    : max_depth_(16)
    , min_depth_(0)
  {}

  PARAMETER(BinaryTreeParameters, int, max_depth);
  PARAMETER(BinaryTreeParameters, int, min_depth);
};

// The BinaryTree class.

template <typename T, typename Axis>
class BinaryTree
{
public:
  typedef BinaryTree<T, Axis> ThisType;
  typedef T NestedType;
  typedef BinaryTreeParameters Parameters;

  typedef BinaryTreeNode<T, Axis::BinaryTreeNumberOfChildren> Node;
  typedef typename Node::Ptr NodePtr;
  typedef typename Node::ConstPtr NodeConstPtr;

  template <typename ParameterType> BinaryTree(const ParameterType& params = ParameterType())
    : root_(0)
  {
    internal::ParameterAdaptor<Parameters> p(params);
    max_depth_ = p.max_depth();
    min_depth_ = p.min_depth();

    Axis::setBinaryTreeSize(size_, 1u << max_depth_);
  }

  virtual ~BinaryTree() {
    clear();
  }

  virtual const Size &getSize() const {
    return size_;
  }

  virtual NodePtr getNode(const GridIndex& original_key, int depth = -1) {
    if (depth == -1) depth = max_depth_;

    // initialize node with root node and relative key with the given key
    if (!root_) root_ = new Node();
    NodePtr node = root_;

    // invert bit at max_depth_-1 as the tree is centered around the origin
    GridIndex key = original_key;
    key[0] ^= (1u << (max_depth_-1));
    key[1] ^= (1u << (max_depth_-1));
    key[2] ^= (1u << (max_depth_-1));

    // walk down the tree...
    for( ; depth > min_depth_; --depth) {
      node = node->get(Axis::getBinaryTreeIndex(key, depth - min_depth_ - 1));
    }

    return node;
  }

  virtual NodeConstPtr getNode(const GridIndex& key, int depth = -1) const {
    if (!root_) return NodeConstPtr();
    return const_cast<NodeConstPtr>(const_cast<ThisType *>(this)->getNode(key, depth));
  }

  virtual T *get(const GridIndex& key, int depth = -1) {
    return getNode(key, depth)->data();
  }

  virtual const T *get(const GridIndex& key, int depth = -1) const {
    return getNode(key, depth)->data();
  }

  virtual void clear() {
    delete root_;
    root_ = 0;
  }

private:
  Node *root_;
  int max_depth_, min_depth_;
  Size size_;
};

} // namespace structure
} // namespace hector_mapping

#endif // HECTOR_MAPPING_STRUCTURE_BINARY_TREE_H
