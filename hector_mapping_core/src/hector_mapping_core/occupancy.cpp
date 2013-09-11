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

#include <hector_mapping_core/map/occupancy.h>

namespace hector_mapping
{

OccupancyGridCell::OccupancyGridCell(const OccupancyParameters &params)
{
  reset(params);
}

OccupancyGridCell::~OccupancyGridCell()
{}

void OccupancyGridCell::setValue(occupancy_t occupancy, const OccupancyParameters &params)
{
  value_ = occupancy;
  if (value_ > params.max_occupancy()) value_ = params.max_occupancy();
  if (value_ < params.min_occupancy()) value_ = params.min_occupancy();
}

bool OccupancyGridCell::isUnknown(const OccupancyParameters &params) const
{
  return !isFree(params) && !isOccupied(params);
}

bool OccupancyGridCell::isFree(const OccupancyParameters &params) const
{
  return value_ < params.threshold_free();
}

bool OccupancyGridCell::isOccupied(const OccupancyParameters &params) const
{
  return value_ > params.threshold_occupied();
}

void OccupancyGridCell::updateOccupied(const OccupancyParameters &params)
{
  setValue(value_ + params.step_occupied(), params);
}

void OccupancyGridCell::updateFree(const OccupancyParameters &params)
{
  setValue(value_ + params.step_occupied(), params);
}

void OccupancyGridCell::reset(const OccupancyParameters &params)
{
  value_ = params.initial_value();
}

} // namespace hector_mapping

