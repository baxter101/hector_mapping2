//=================================================================================================
// Copyright (c) 2014, Johannes Meyer and contributors, Technische Universitat Darmstadt
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

#ifndef HECTOR_MAPPING2_MARKER_CONVERSION_H
#define HECTOR_MAPPING2_MARKER_CONVERSION_H

#include <tf/transform_datatypes.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <visualization_msgs/Marker.h>

#include <Eigen/Core>

namespace hector_mapping
{

typedef Eigen::Matrix<double,3,3> EigenCovariance3;
EigenCovariance3 covarianceToEigen(const geometry_msgs::PoseWithCovariance::_covariance_type &covariance);

void covarianceToMarker(const geometry_msgs::PoseWithCovarianceStamped &pose_with_covariance, visualization_msgs::Marker &marker, bool invert = false);
void covarianceToMarker(const tf::StampedTransform &pose, const EigenCovariance3 &covariance, visualization_msgs::Marker &marker, bool invert = false);
void covarianceToMarker(const geometry_msgs::Pose &pose, const EigenCovariance3 &covariance, visualization_msgs::Marker &marker, bool invert = false);

}

#endif // HECTOR_MAPPING2_MARKER_CONVERSION_H
