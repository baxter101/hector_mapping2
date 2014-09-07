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

#include <hector_mapping_core/structure/structure.h>
#include <hector_mapping_core/internal/axes.h>
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
    (*children_)[i] = new ThisType();
  }

  void prune(const size_type i) {
    static Children all_zero = {{ 0, 0, 0, 0, 0, 0, 0, 0 }};
    if (children_) {
      delete (*children_)[i];
      (*children_)[i] = 0;
      if (*children_ == all_zero) {
        delete children_;
        children_ = 0;
      }
    }
  }

  bool hasChildren() const { return children_; }

  Ptr get(const size_type i, bool _expand = true) { if (_expand) expand(i); return children_ ? (*children_)[i] : 0; }
  ConstPtr get(const size_type i) const { return children_ ? (*children_)[i] : 0; }

  T* data() { return &data_; }
  const T* data() const { return &data_; }

  T& operator*() { return data_; }
  const T& operator*() const { return data_; }

private:
  T data_;
  Children *children_;
};

// The BinaryTree class.

template <typename T, typename Axes>
class BinaryTree : public StructureBase
{
public:
  typedef BinaryTree<T, Axes> ThisType;
  typedef T NestedType;

  typedef BinaryTreeNode<T, Axes::BinaryTreeNumberOfChildren> Node;
  typedef typename Node::Ptr NodePtr;
  typedef typename Node::ConstPtr NodeConstPtr;

  BinaryTree(const Parameters& params = Parameters())
    : StructureBase(params)
    , root_(0)
  {
//    params("max_depth", max_depth_).default_value(16);
    params("max_depth", max_depth_).default_value(10);
    params("map_size", size_);

    Size size;
    Axes::setBinaryTreeSize(size, 1u << max_depth_);
    size_ = size;

    clear();
  }

  virtual ~BinaryTree() {
    delete root_;
  }

  virtual const Size &getSize() const {
    return size_;
  }

  bool setExtends(const GridIndex &min, const GridIndex &max) {
    // todo: resize binary tree dynamically
    return false;
  }

  virtual NodePtr getNode(const GridIndex& original_key, int depth = -1, bool _expand = true) {
    if (depth < 0) depth = max_depth_;

    // initialize node with root node
    NodePtr node = root_;
    NodePtr found = node;
    int current_depth = 0;

    // invert bit at max_depth_-1 as the tree is centered around the origin
    GridIndex key = original_key;
    key[0] ^= (1u << (max_depth_-1));
    key[1] ^= (1u << (max_depth_-1));
    key[2] ^= (1u << (max_depth_-1));

    // walk down the tree...
    for( ; current_depth < depth; ++current_depth) {
      if (!node) break;
      node = node->get(Axes::getBinaryTreeIndex(key, max_depth_ - current_depth - 1), _expand);
      if (node) found = node;
    }

    return found;
  }

  virtual NodeConstPtr getNode(const GridIndex& key, int depth = -1) const {
    return const_cast<NodeConstPtr>(const_cast<ThisType *>(this)->getNode(key, depth, false));
  }

  T *get(const GridIndex& key, int level = 0) {
    NodePtr node = (level < 0 ? getNode(key, level, level != level::SEARCH) : getNode(key, max_depth_ - level));
    if (!node) return 0;
    return node->data();
  }

  const T *get(const GridIndex& key, int level = 0) const {
    NodeConstPtr node = (level < 0 ? getNode(key, level) : getNode(key, max_depth_ - level));
    if (!node) return 0;
    return node->data();
  }

  virtual void clear() {
    delete root_;
    root_ = new Node();
  }

private:
  Node *root_;
  int max_depth_;
  Size size_;
};

} // namespace structure
} // namespace hector_mapping

#endif // HECTOR_MAPPING_STRUCTURE_BINARY_TREE_H
