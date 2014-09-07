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
#include <hector_mapping_core/scan.h>

#include <tf_conversions/tf_eigen.h>
#include <limits>
#include <ros/console.h>

namespace hector_mapping
{

OccupancyParameters::OccupancyParameters()
  : min_occupancy_(std::numeric_limits<occupancy_t>::min())
  , max_occupancy_(std::numeric_limits<occupancy_t>::max())
  , initial_value_(0)
  , step_occupied_(1)
  , step_free_(-1)
  , threshold_occupied_(0)
  , threshold_free_(0)
  , logodd_scale_factor_(10.0)
{}

OccupancyParameters &OccupancyParameters::Default() {
  static OccupancyParameters s_default;
  return s_default;
}

occupancy_t OccupancyParameters::getOccupancy(float probability) const {
  float logodd = getLogOdd(probability);
  int occupancy = floor(logodd * logodd_scale_factor_ + .5f);
  if (occupancy < min_occupancy_) occupancy = min_occupancy_;
  if (occupancy > max_occupancy_) occupancy = max_occupancy_;
  return static_cast<occupancy_t>(occupancy);
}

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
  setValue(value_ + params.step_free(), params);
}

void OccupancyGridCell::undoUpdateFree(const OccupancyParameters &params)
{
  setValue(value_ - params.step_free(), params);
}

void OccupancyGridCell::reset(const OccupancyParameters &params)
{
  value_ = params.initial_value();
}

OccupancyGridMapBase::OccupancyGridMapBase(const Parameters& _params)
  : GridMapBase(_params)
  , occupancy_parameters_(params().add<OccupancyParameters>("occupancy", OccupancyParameters::Default()))
  , empty_(true)
{}

OccupancyGridMapBase::~OccupancyGridMapBase()
{}

void OccupancyGridMapBase::reset()
{
  empty_ = true;
}

OccupancyGridMapBase::ValueType OccupancyGridMapBase::getValue(const GridIndex& key, int level) const {
  const OccupancyGridCell *occupancy = getOccupancy(key, level);
  if (!occupancy) return std::numeric_limits<ValueType>::quiet_NaN();
  return occupancy->getProbability(getOccupancyParameters());
}


bool OccupancyGridMapBase::insert(const Scan &scan, const tf::Transform &transform)
{
  Eigen::Affine3d eigen_transform;
  tf::transformTFToEigen(transform, eigen_transform);
  return insert(scan, eigen_transform);
}

bool OccupancyGridMapBase::insert(const Scan &scan, const Eigen::Affine3d &transform)
{
  // initialize sets to remember free and occupied cells
  GridIndexSet updated_occupied;
  GridIndexSet updated_free;

  // get origin
  GridIndex origin = toGridIndex(transform.translation().cast<float_t>());

  // update cells for each scan point using Bresenham's line algorithm, going from the endpoint to the origin
  Scan::iterator iterator = scan.begin();
  while(iterator != scan.end()) {
    GridIndex endpoint = toGridIndex((transform * (*iterator).cast<double>()).cast<float_t>());

    index_t abs2[3];
    abs2[0] = 2 * abs(endpoint[0] - origin[0]);
    abs2[1] = 2 * abs(endpoint[1] - origin[1]);
    abs2[2] = 2 * abs(endpoint[2] - origin[2]);

    diff_t step[3];
    step[0]  = (origin[0] < endpoint[0]) ? 1 : -1;
    step[1]  = (origin[1] < endpoint[1]) ? 1 : -1;
    step[2]  = (origin[2] < endpoint[2]) ? 1 : -1;

    int dominant = 0;
    if (abs2[1] > abs2[dominant]) dominant = 1;
    if (abs2[2] > abs2[dominant]) dominant = 2;

    diff_t delta[3];
    delta[0] = abs2[0] - abs2[dominant] / 2;
    delta[1] = abs2[1] - abs2[dominant] / 2;
    delta[2] = abs2[2] - abs2[dominant] / 2;
    delta[dominant] = 0;

    GridIndex current = endpoint;

    // mark endpoint as occupied
    updateOccupied(endpoint, updated_occupied, updated_free);

    while(1) {
      if (current[dominant] == origin[dominant]) break;

      if ((dominant != 0) && (delta[0] >= 0)) {      // move along x
        current[0] -= step[0];
        delta[0]   -= abs2[dominant];
      }

      if ((dominant != 1) && (delta[1] >= 0)) {      // move along y
        current[1] -= step[1];
        delta[1]   -= abs2[dominant];
      }

      if ((dominant != 2) && (delta[2] >= 0)) {      // move along z
        current[2] -= step[2];
        delta[2]   -= abs2[dominant];
      }

      // always move along dominant axis (just reset delta[dominant] = 0)
      current[dominant] -= step[dominant];
      delta[0] += abs2[0];
      delta[1] += abs2[1];
      delta[2] += abs2[2];

      // mark current cell as free
      updateFree(current, updated_occupied, updated_free);
    }

    // go to next endpoint
    ++iterator;
  }

  ROS_DEBUG("Inserted scan with %lu endpoints. %lu cells have been updated as occupied and %lu as free.", scan.size(), updated_occupied.size(), updated_free.size());
  if (updated_occupied.size() > 0) empty_ = false;
}

void OccupancyGridMapBase::updateFree(const GridIndex &key, GridIndexSet &occupied, GridIndexSet &free)
{
  if (!growMinExtends(key)) return;

  // skip update if this cell has already been marked as occupied or free
  if (occupied.count(key)) return;
  if (free.count(key)) return;

  // get cell
  OccupancyGridCell *cell = getOccupancy(key);

  // update occupancy
  cell->updateFree(getOccupancyParameters());
  free.insert(key);
}

void OccupancyGridMapBase::updateOccupied(const GridIndex &key, GridIndexSet &occupied, GridIndexSet &free)
{
  if (!growMinExtends(key)) return;

  // skip update if this cell has already been marked as occupied
  if (occupied.count(key)) return;

  // get cell
  OccupancyGridCell *cell = getOccupancy(key);

  // undo the free update if this cell has been marked as free
  if (free.count(key)) {
    cell->undoUpdateFree(getOccupancyParameters());
    free.erase(key);
  }

  // update occupancy
  cell->updateOccupied(getOccupancyParameters());
  occupied.insert(key);
}

} // namespace hector_mapping

