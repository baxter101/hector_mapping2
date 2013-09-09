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

#include <hector_mapping_core/map/cell.h>
#include <hector_mapping_core/internal/macros.h>

#include <limits>

namespace hector_mapping
{

typedef int occupancy_t;

class OccupancyParameters
{
  PARAMETER(OccupancyParameters, occupancy_t, min_occupancy);
  PARAMETER(OccupancyParameters, occupancy_t, max_occupancy);
  PARAMETER(OccupancyParameters, occupancy_t, initial_value);
  PARAMETER(OccupancyParameters, occupancy_t, step_occupied);
  PARAMETER(OccupancyParameters, occupancy_t, step_free);
  PARAMETER(OccupancyParameters, occupancy_t, threshold_occupied);
  PARAMETER(OccupancyParameters, occupancy_t, threshold_free);

public:
  OccupancyParameters()
    : min_occupancy_(std::numeric_limits<occupancy_t>::min())
    , max_occupancy_(std::numeric_limits<occupancy_t>::max())
    , initial_value_(0)
    , step_occupied_(1)
    , step_free_(-1)
    , threshold_occupied_(0)
    , threshold_free_(0)
  {}

  static OccupancyParameters &Default() {
    static OccupancyParameters s_default;
    return s_default;
  }
};

class OccupancyGridCell : public GridCellBase
{
public:
  typedef OccupancyParameters Parameters;

  OccupancyGridCell(const OccupancyParameters &parameters = OccupancyParameters::Default());
  ~OccupancyGridCell();

  occupancy_t getOccupancy() const { return value_; }
  void setOccupancy(occupancy_t occupancy, const OccupancyParameters &parameters = OccupancyParameters::Default());

  bool isUnknown(const OccupancyParameters &parameters = OccupancyParameters::Default()) const;
  bool isFree(const OccupancyParameters &parameters = OccupancyParameters::Default()) const;
  bool isOccupied(const OccupancyParameters &parameters = OccupancyParameters::Default()) const;

  void updateOccupied(const OccupancyParameters &parameters = OccupancyParameters::Default());
  void updateFree(const OccupancyParameters &parameters = OccupancyParameters::Default());

  void reset(const OccupancyParameters &parameters = OccupancyParameters::Default());

private:
  occupancy_t value_;
};

typedef GridMap<OccupancyGridCell> OccupancyGridMap;

} // namespace hector_mapping

#endif // HECTOR_MAPPING_MAP_OCCUPANCY_H
