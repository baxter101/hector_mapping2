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


#ifndef HECTOR_MAPPING2_MAP_OCCUPANCY_H
#define HECTOR_MAPPING2_MAP_OCCUPANCY_H

#include <hector_mapping2_core/map.h>
#include <hector_mapping2_core/map/cell.h>
#include <hector_mapping2_core/internal/macros.h>

namespace hector_mapping
{

typedef int16_t occupancy_t;
typedef float probability_t;

class OccupancyParameters
{
  PARAMETER(OccupancyParameters, occupancy_t, min_occupancy);
  PARAMETER(OccupancyParameters, occupancy_t, max_occupancy);
  PARAMETER(OccupancyParameters, occupancy_t, step_occupied);
  PARAMETER(OccupancyParameters, occupancy_t, step_free);
  PARAMETER(OccupancyParameters, occupancy_t, threshold_occupied);
  PARAMETER(OccupancyParameters, occupancy_t, threshold_free);
  PARAMETER(OccupancyParameters, probability_t, logodd_scale_factor);

public:
  OccupancyParameters();
  static OccupancyParameters &Default();

  probability_t getProbability(occupancy_t occupancy) const {
    return probabilityFromLogOdd(occupancy / logodd_scale_factor_);
  }

  probability_t getLogOdd(occupancy_t occupancy) const {
    return static_cast<probability_t>(occupancy) / logodd_scale_factor_;
  }

  occupancy_t getOccupancy(probability_t probability) const;
  occupancy_t applyBounds(int occupancy) const;

  static probability_t probabilityFromLogOdd(probability_t logodd) {
    probability_t odd = exp(logodd);
    return odd / (odd + 1.0f);
  }

  static probability_t logOddFromProbability(probability_t probability) {
    probability_t odd = probability / (1.0f - probability);
    return log(odd);
  }
};

class OccupancyGridCell : public GridCellBase
{
public:
  typedef probability_t ValueType;

  OccupancyGridCell();
  ~OccupancyGridCell();

  occupancy_t getOccupancy() const { return occupancy_; }
  void setOccupancy(occupancy_t occupancy, const OccupancyParameters &parameters);

  ValueType getValue(const OccupancyParameters &parameters = OccupancyParameters::Default()) const { return getProbability(parameters); }
  void setValue(ValueType probability, const OccupancyParameters &parameters = OccupancyParameters::Default()) { setProbability(probability, parameters); }

  probability_t getProbability(const OccupancyParameters &parameters) const { return parameters.getProbability(occupancy_); }
  probability_t getLogOdd(const OccupancyParameters &parameters) const { return parameters.getLogOdd(occupancy_); }
  void setProbability(probability_t probability, const OccupancyParameters &parameters) { occupancy_ = parameters.getOccupancy(probability); }

  bool isUnknown(const OccupancyParameters &parameters) const;
  bool isFree(const OccupancyParameters &parameters) const;
  bool isOccupied(const OccupancyParameters &parameters) const;

  void updateOccupied(const OccupancyParameters &parameters);
  void updateFree(const OccupancyParameters &parameters);
  void undoUpdateFree(const OccupancyParameters &parameters);

  void reset();

private:
  occupancy_t occupancy_;
};

class OccupancyGridMapBase : public GridMapBase
{
public:
  typedef OccupancyGridCell::ValueType ValueType;
  typedef boost::unordered_map<CacheKey, ValueType> Cache;
  typedef boost::shared_ptr<Cache> CachePtr;

  OccupancyGridMapBase(const Parameters& params = Parameters());
  virtual ~OccupancyGridMapBase();

  virtual OccupancyGridCell *get(const GridIndex& key, int level = 0) = 0;
  virtual const OccupancyGridCell *get(const GridIndex& key, int level = 0) const = 0;

  virtual OccupancyGridCell *get(const Point& point, int level = 0)               { return get(toGridIndex(point), level); }
  virtual const OccupancyGridCell *get(const Point& point, int level = 0) const   { return get(toGridIndex(point), level); }

  virtual ValueType getValue(const GridIndex& key, int level = 0) const;
  virtual ValueType getValue(const Point& point, int level = 0) const { return getValue(toGridIndex(point), level); }

  OccupancyParameters& getOccupancyParameters() { return occupancy_parameters_; }
  const OccupancyParameters& getOccupancyParameters() const { return occupancy_parameters_; }

  virtual bool insert(const Scan& scan, const tf::Transform& transform);
  virtual bool insert(const Scan& scan, const Eigen::Affine3d& transform);

  virtual CachePtr cache() const = 0;

private:
  void updateOccupied(const GridIndex& key, GridIndexSet& occupied, GridIndexSet& free);
  void updateFree(const GridIndex& key, GridIndexSet& occupied, GridIndexSet& free);

  OccupancyParameters &occupancy_parameters_;
};

template <typename OccupancyCellType>
class OccupancyGridMap : public GridMap<OccupancyCellType,OccupancyGridMapBase>
{
public:
  using typename GridMap<OccupancyCellType,OccupancyGridMapBase>::BaseType;
  using typename GridMap<OccupancyCellType,OccupancyGridMapBase>::CellType;
  typedef typename OccupancyCellType::ValueType ValueType;

  OccupancyGridMap(const Parameters& _params = Parameters())
    : GridMap<OccupancyCellType,OccupancyGridMapBase>(_params)
  {
  }
  virtual ~OccupancyGridMap() {}

//  using GridMap<OccupancyCellType,OccupancyGridMapBase>::params;
//  using GridMap<OccupancyCellType,OccupancyGridMapBase>::operator();
//  using GridMap<OccupancyCellType,OccupancyGridMapBase>::get;

  virtual ValueType getValue(const GridIndex& key, int level = 0) const { return OccupancyGridMapBase::getValue(key, level); }
  virtual ValueType getValue(const Point& point, int level = 0) const   { return OccupancyGridMapBase::getValue(point, level); }
};

} // namespace hector_mapping

#endif // HECTOR_MAPPING2_MAP_OCCUPANCY_H
