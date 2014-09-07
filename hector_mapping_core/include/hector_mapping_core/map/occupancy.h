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


#ifndef HECTOR_MAPPING_MAP_OCCUPANCY_H
#define HECTOR_MAPPING_MAP_OCCUPANCY_H

#include <hector_mapping_core/map.h>
#include <hector_mapping_core/map/cell.h>
#include <hector_mapping_core/internal/macros.h>

#include <limits>
#include <set>

namespace hector_mapping
{

typedef int16_t occupancy_t;

class OccupancyParameters
{
  PARAMETER(OccupancyParameters, occupancy_t, min_occupancy);
  PARAMETER(OccupancyParameters, occupancy_t, max_occupancy);
  PARAMETER(OccupancyParameters, occupancy_t, initial_value);
  PARAMETER(OccupancyParameters, occupancy_t, step_occupied);
  PARAMETER(OccupancyParameters, occupancy_t, step_free);
  PARAMETER(OccupancyParameters, occupancy_t, threshold_occupied);
  PARAMETER(OccupancyParameters, occupancy_t, threshold_free);
  PARAMETER(OccupancyParameters, float, logodd_scale_factor);

public:
  OccupancyParameters();
  static OccupancyParameters &Default();

  float getProbability(occupancy_t occupancy) const {
    return probabilityFromLogOdd(occupancy / logodd_scale_factor_);
  }

  float getLogOdd(occupancy_t occupancy) const {
    return occupancy / logodd_scale_factor_;
  }

  occupancy_t getOccupancy(float probability) const;

  static float probabilityFromLogOdd(float logodd) {
    float odd = exp(logodd);
    return odd / (odd + 1.0f);
  }

  static float logOddFromProbability(float probability) {
    float odd = probability / (1.0f - probability);
    return log(odd);
  }
};

class OccupancyGridCell : public GridCellBase
{
public:
  OccupancyGridCell(const OccupancyParameters &parameters = OccupancyParameters::Default());
  ~OccupancyGridCell();

  occupancy_t getValue() const { return value_; }
  void setValue(occupancy_t occupancy, const OccupancyParameters &parameters = OccupancyParameters::Default());

  bool isUnknown(const OccupancyParameters &parameters = OccupancyParameters::Default()) const;
  bool isFree(const OccupancyParameters &parameters = OccupancyParameters::Default()) const;
  bool isOccupied(const OccupancyParameters &parameters = OccupancyParameters::Default()) const;

  void updateOccupied(const OccupancyParameters &parameters = OccupancyParameters::Default());
  void updateFree(const OccupancyParameters &parameters = OccupancyParameters::Default());
  void undoUpdateFree(const OccupancyParameters &parameters = OccupancyParameters::Default());

  void reset(const OccupancyParameters &parameters = OccupancyParameters::Default());

  float getProbability(const OccupancyParameters &parameters = OccupancyParameters::Default()) const { return parameters.getProbability(getValue()); }
  float getLogOdd(const OccupancyParameters &parameters = OccupancyParameters::Default()) const { return parameters.getLogOdd(getValue()); }

private:
  occupancy_t value_;
};

class OccupancyGridMapBase : public GridMapBase
{
public:
  typedef float ValueType;

  OccupancyGridMapBase(const Parameters& params = Parameters());
  virtual ~OccupancyGridMapBase();

  virtual OccupancyGridCell *getOccupancy(const GridIndex& key, int level = 0) = 0;
  virtual const OccupancyGridCell *getOccupancy(const GridIndex& key, int level = 0) const = 0;

  virtual OccupancyGridCell *getOccupancy(const Point& point, int level = 0)               { return getOccupancy(toGridIndex(point), level); }
  virtual const OccupancyGridCell *getOccupancy(const Point& point, int level = 0) const   { return getOccupancy(toGridIndex(point), level); }

  virtual ValueType getValue(const GridIndex& key, int level = 0) const;
  virtual ValueType getValue(const Point& point, int level = 0) const { return getValue(toGridIndex(point), level); }

  OccupancyParameters& getOccupancyParameters() { return occupancy_parameters_; }
  const OccupancyParameters& getOccupancyParameters() const { return occupancy_parameters_; }

  virtual void reset();
  virtual bool empty() { return empty_; }

  virtual bool insert(const Scan& scan, const tf::Transform& transform);
  virtual bool insert(const Scan& scan, const Eigen::Affine3d& transform);

private:
  void updateOccupied(const GridIndex& key, GridIndexSet& occupied, GridIndexSet& free);
  void updateFree(const GridIndex& key, GridIndexSet& occupied, GridIndexSet& free);

  OccupancyParameters &occupancy_parameters_;
  bool empty_;
};

template <typename OccupancyCellType>
class OccupancyGridMap : public GridMap<OccupancyCellType,OccupancyGridMapBase>
{
public:
  using GridMap<OccupancyCellType,OccupancyGridMapBase>::BaseType;
  using GridMap<OccupancyCellType,OccupancyGridMapBase>::CellType;
  using OccupancyGridMapBase::ValueType;

  OccupancyGridMap(const Parameters& _params = Parameters())
    : GridMap<OccupancyCellType,OccupancyGridMapBase>(_params)
  {
  }
  virtual ~OccupancyGridMap() {}

  using GridMap<OccupancyCellType,OccupancyGridMapBase>::params;
  using GridMap<OccupancyCellType,OccupancyGridMapBase>::operator();
  using GridMap<OccupancyCellType,OccupancyGridMapBase>::get;
  using GridMap<OccupancyCellType,OccupancyGridMapBase>::getOccupancy;

  virtual OccupancyGridCell *getOccupancy(const GridIndex& key, int level = 0)             { return GridMapBase::isValid(key) ? get(key, level) : 0; }
  virtual const OccupancyGridCell *getOccupancy(const GridIndex& key, int level = 0) const { return GridMapBase::isValid(key) ? get(key, level) : 0; }
};

} // namespace hector_mapping

#endif // HECTOR_MAPPING_MAP_OCCUPANCY_H
