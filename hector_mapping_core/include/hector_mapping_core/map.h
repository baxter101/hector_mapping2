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


#ifndef HECTOR_MAPPING_MAP_H
#define HECTOR_MAPPING_MAP_H

#include <hector_mapping_core/types.h>
#include <hector_mapping_core/internal/macros.h>
#include <hector_mapping_core/structure/get_cell.h>
#include <hector_mapping_core/structure/array.h>

#include <boost/array.hpp>
#include <Eigen/Geometry>

#include <std_msgs/Header.h>

namespace hector_mapping
{

class MapBase
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  template <typename ParameterType> MapBase(const ParameterType& params = ParameterType())
  {
    offset_ = params.offset();
  }
  virtual ~MapBase() {}

  virtual void getExtends(Point &min, Point &max) = 0;
  virtual bool setExtends(const Point &min, const Point &max) = 0;
  virtual bool growMinExtends(const Point &point) = 0;

  const Point& getOffset() const { return offset_; }
  virtual void setOffset(const Point &offset) { offset_ = offset; }

  virtual void clear() = 0;
  virtual void insertScan(const Scan& scan, const Transform& transform) {}

  const Header& getHeader() const { return header_; }
  Header& getHeader() { return header_; }

protected:
  Point offset_;
  std_msgs::Header header_;
};

class GridMapParameters : virtual public ArrayParameters
{
  PARAMETER(GridMapParameters, Point, offset);
  PARAMETER(GridMapParameters, Resolution, resolution);

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  GridMapParameters()
    : offset_(0.0, 0.0 ,0.0)
    , resolution_(0.01, 0.01, 0.01)
  {}
  virtual ~GridMapParameters() {}
};

class GridMapBase : public MapBase
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  template <typename ParameterType> GridMapBase(const ParameterType& params = ParameterType())
    : MapBase(params)
  {
    resolution_ = params.resolution();
  }

  virtual ~GridMapBase() {}

  virtual const Size& getSize() const = 0;
  virtual void getExtends(Point &min, Point &max);
  virtual bool setExtends(const Point &min, const Point &max);
  virtual bool growMinExtends(const Point &point);

  const Resolution& getResolution() const { return resolution_; }
  virtual void setResolution(const Resolution& resolution) { resolution_ = resolution; }

  virtual index_t toGridIndexAxis(float_t coordinates, int axis) const;
  virtual float_t toPointAxis(index_t index, int axis) const;

  virtual GridIndex toGridIndex(const Point &point) const;
  virtual Point toPoint(const GridIndex &index) const;

  enum { X = 0, Y = 1, Z = 2 };
  virtual GridIndex getNeighbourIndex(const GridIndex &key, unsigned int axis, int step = 1) const;

protected:
  Resolution resolution_;
};

template <typename _CellType, typename _BaseType = GridMapBase>
class GridMap : public _BaseType
{
public:
  typedef _CellType CellType;
  typedef _BaseType BaseType;

  struct Parameters : public GridMapParameters, public CellType::Parameters {};
  typedef typename CellType::Parameters CellParameters;

  template <typename ParameterType> GridMap(const ParameterType& params = ParameterType())
    : _BaseType(params)
    , cell_params_(internal::ParameterAdaptor<CellParameters>(params))
  {
  }
  virtual ~GridMap() {}

  virtual CellType *get(const GridIndex& key) = 0;
  virtual const CellType *get(const GridIndex& key) const = 0;

  // convenience access functions
  CellType& operator()(const GridIndex& key)             { return *get(key); }
  const CellType& operator()(const GridIndex& key) const { return *get(key); }
  CellType& operator()(const Point& point)               { return *get(this->toGridIndex(point)); }
  const CellType& operator()(const Point& point) const   { return *get(this->toGridIndex(point)); }

  // get cell parameter struct
  const CellParameters& getCellParameters() const { return cell_params_; }

private:
  CellParameters cell_params_;
};

template <typename GridMapType, typename Structure>
class GridMapImpl : public GridMapType, public Structure
{
public:
  typedef typename GridMapType::CellType CellType;
  struct Parameters : public GridMapType::Parameters, virtual public Structure::Parameters {};

  template <typename ParameterType> GridMapImpl(const ParameterType& params = ParameterType())
    : GridMapType(params)
    , Structure(params)
  {}
  virtual ~GridMapImpl() {}

  virtual void clear() { Structure::clear(); }
  virtual const Size &getSize() const { return Structure::getSize(); }

  virtual typename Structure::NestedType *getNext(const GridIndex& key) { return Structure::get(key); }
  virtual const typename Structure::NestedType *getNext(const GridIndex& key) const { return Structure::get(key); }

  virtual CellType *get(const GridIndex& key) { return GetCell<CellType,Structure>::get(*this, key); }
  virtual const CellType *get(const GridIndex& key) const { return GetCell<CellType,Structure>::get(*this, key); }

  // convenience access functions
  CellType& operator()(const GridIndex& key)             { return *get(key); }
  const CellType& operator()(const GridIndex& key) const { return *get(key); }
  CellType& operator()(const Point& point)               { return *get(this->toGridIndex(point)); }
  const CellType& operator()(const Point& point) const   { return *get(this->toGridIndex(point)); }
};


} // namespace hector_mapping

#endif // HECTOR_MAPPING_MAP_H
