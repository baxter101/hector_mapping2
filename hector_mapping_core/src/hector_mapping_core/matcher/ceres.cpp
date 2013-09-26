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

#include <hector_mapping_core/matcher/ceres.h>
#include <hector_mapping_core/scan.h>

#ifdef HAVE_CERES
#include "solver.hpp"

namespace hector_mapping {
namespace matcher {

Ceres::Ceres(const Parameters &params)
  : ScanMatcher(params)
{
}

Ceres::~Ceres()
{}

bool Ceres::match(const OccupancyGridMapBase &map, const Scan &scan, MatchType type)
{
  // acquire a shared lock
  MapBase::SharedLock lock(map.getLock());
  tf::Transform result = transform_;
  covariance_valid_ = false;

  switch(type) {
    case MATCH_2D:
      {
        SolverPtr solver = Solver::initialize<MATCH_2D>(map, scan, params_, transform_);
        if (!solver->solve(result)) return false;
        // covariance_valid_ = solver->getCovariance(covariance_);
        covariance_valid_ = solver->getInformationMatrix(covariance_);
      }
      break;

    case MATCH_3D:
      {
        SolverPtr solver = Solver::initialize<MATCH_3D>(map, scan, params_, transform_);
        if (!solver->solve(result)) return false;
        // covariance_valid_ = solver->getCovariance(covariance_);
        covariance_valid_ = solver->getInformationMatrix(covariance_);
      }
      break;

    default:
      return false;
  }

  transform_.stamp_ = scan.getStamp();
  transform_.setData(result);
  return true;
}

double Ceres::evaluate(const OccupancyGridMapBase &map, const Scan &scan, const tf::Transform &pose, std::string *resultString, MatchType type) const
{
  // acquire a shared lock
  MapBase::SharedLock lock(map.getLock());
  double cost;

  switch(type) {
    case MATCH_2D:
      {
        SolverPtr solver = Solver::initialize<MATCH_2D>(map, scan, params_, pose);
        cost = solver->evaluate();
        if (resultString) *resultString = solver->evaluateDebug();
      }
      break;

    case MATCH_3D:
      {
        SolverPtr solver = Solver::initialize<MATCH_3D>(map, scan, params_, pose);
        cost = solver->evaluate();
        if (resultString) *resultString = solver->evaluateDebug();
      }
      break;

    default:
      return -1.0;
  }

  return cost;
}

} // namespace matcher
} // namespace hector_mapping

#endif // HAVE_CERES
