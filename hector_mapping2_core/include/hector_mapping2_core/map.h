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


#ifndef HECTOR_MAPPING2_MAP_H
#define HECTOR_MAPPING2_MAP_H

#include <hector_mapping2_core/types.h>
#include <hector_mapping2_core/parameters.h>
#include <hector_mapping2_core/internal/macros.h>
#include <hector_mapping2_core/structure/get_cell.h>
#include <hector_mapping2_core/structure/array.h>

#include <boost/array.hpp>
#include <Eigen/Geometry>

#include <boost/thread/shared_mutex.hpp>

namespace hector_mapping
{

struct MapFactory
{
public:
  MapFactory(const Parameters& params = Parameters());
  template <typename MapType> boost::shared_ptr<MapType> create(const std::string& type) { return boost::dynamic_pointer_cast<MapType>((*this)(type)); }

  // implemented in types.cpp
  MapBasePtr operator()(const std::string& type);

private:
  Parameters params_;
};

class MapBase
{
public:
  MapBase(const Parameters& params = Parameters());
  virtual ~MapBase() {}

  virtual Parameters& params() { return params_; }
  virtual const Parameters& params() const { return params_; }

  virtual void getExtends(Point &min, Point &max) const = 0;
  virtual bool setExtends(const Point &min, const Point &max) = 0;
  virtual bool growMinExtends(const Point &point);

  const Point& getOffset() const { return offset_; }
  virtual void setOffset(const Point &offset) { offset_ = offset; }

  virtual bool empty() { return empty_; }
  virtual void reset();

  const std_msgs::Header& getHeader() const { return header_; }
  std_msgs::Header& getHeader() { return header_; }

  typedef boost::shared_mutex Mutex;
  typedef boost::shared_lock<boost::shared_mutex> SharedLock;
  typedef boost::unique_lock<boost::shared_mutex> UniqueLock;
  SharedLock getLock() const { return SharedLock(mutex_); }
  UniqueLock getWriteLock() { return UniqueLock(mutex_); }

protected:
  virtual void updated();

private:
  Parameters params_;
  mutable Mutex mutex_;

protected:
  Parameter<Point> offset_;
  std_msgs::Header header_;
  bool empty_;
};

class GridMapBase : public MapBase
{
public:
  GridMapBase(const Parameters& params = Parameters());
  virtual ~GridMapBase() {}

  virtual const Size& getSize() const = 0;

  virtual void getExtends(Point &min, Point &max) const;
  virtual void getExtends(GridIndex &min, GridIndex &max) const;
  virtual bool setExtends(const Point &min, const Point &max);
  virtual bool setExtends(const GridIndex &min, const GridIndex &max) = 0;
  using MapBase::growMinExtends;
  virtual bool growMinExtends(const GridIndex &index);

  const Resolution& getResolution(int level = 0) const { return resolution_; }
  virtual void setResolution(const Resolution& resolution) { resolution_ = resolution; }

  virtual index_t toGridIndexAxis(float_t coordinates, int axis) const;
  virtual float_t toPointAxis(index_t index, int axis) const;

  virtual GridIndex toGridIndex(const Point &point) const;
  virtual Point toPoint(const GridIndex &index) const;

  virtual bool isValid(const GridIndex &index) const;

  enum { X = 0, Y = 1, Z = 2 };
  virtual GridIndex getNeighbourIndex(const GridIndex &key, unsigned int axis, int step = 1) const;

protected:
  Parameter<Resolution> resolution_;
};

template <typename _CellType, typename _BaseType = GridMapBase>
class GridMap : public _BaseType
{
public:
  typedef _BaseType BaseType;
  typedef _CellType CellType;
  typedef typename _CellType::ValueType ValueType;

  typedef boost::unordered_map<CacheKey, ValueType> Cache;
  typedef boost::shared_ptr<Cache> CachePtr;

  GridMap(const Parameters& params = Parameters())
    : _BaseType(params)
  {}
  virtual ~GridMap() {}

  virtual CellType *get(const GridIndex& key, int level = 0) = 0;
  virtual const CellType *get(const GridIndex& key, int level = 0) const = 0;

  virtual CellType *get(const Point& point, int level = 0)             { return get(this->toGridIndex(point), level); }
  virtual const CellType *get(const Point& point, int level = 0) const { return get(this->toGridIndex(point), level); }

  virtual ValueType getValue(const GridIndex& key, int level = 0) const { return get(key, level)->getValue(); }
  virtual ValueType getValue(const Point& point, int level = 0) const   { return get(point, level)->getValue(); }

  CachePtr cache() const {
    if (!cache_) cache_.reset(new Cache());
    return cache_;
  }

  void updated() {
    if (cache_) cache_->clear();
    BaseType::updated();
  }

  // convenience access functions
  CellType& operator()(const GridIndex& key, int level = 0)             { return *get(key, level); }
  const CellType& operator()(const GridIndex& key, int level = 0) const { return *get(key, level); }
  CellType& operator()(const Point& point, int level = 0)               { return *get(point, level); }
  const CellType& operator()(const Point& point, int level = 0) const   { return *get(point, level); }

private:
  mutable CachePtr cache_;
};

template <typename GridMapType, typename Structure>
class GridMapImpl : public GridMapType, public Structure
{
public:
  typedef typename GridMapType::CellType CellType;

  GridMapImpl(const Parameters& params = Parameters())
    : GridMapType(params)
    , Structure(params)
  {}
  virtual ~GridMapImpl() {}

  using GridMapType::operator();

  virtual void reset() { GridMapType::reset(); Structure::clear(); }

  virtual const Size &getSize() const { return Structure::getSize(); }

  using GridMapBase::setExtends;
  virtual bool setExtends(const GridIndex &min, const GridIndex &max) { return Structure::setExtends(min, max); }

  virtual typename Structure::NestedType *getNested(const GridIndex& key, int level = 0) { return Structure::get(key, level); }
  virtual const typename Structure::NestedType *getNested(const GridIndex& key, int level = 0) const { return Structure::get(key, level); }

  using GridMapType::get;
  virtual CellType *get(const GridIndex& key, int level = 0) { return GetCell<CellType,Structure>::get(*this, key, level); }
  virtual const CellType *get(const GridIndex& key, int level = 0) const { return GetCell<CellType,Structure>::get(*this, key, level); }
};

} // namespace hector_mapping

#endif // HECTOR_MAPPING2_MAP_H
