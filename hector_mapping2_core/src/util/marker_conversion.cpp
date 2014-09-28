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

#include <hector_mapping2_core/util/marker_conversion.h>
#include <Eigen/Eigenvalues>

using visualization_msgs::Marker;
using geometry_msgs::Pose;
using geometry_msgs::PoseWithCovariance;
using geometry_msgs::PoseWithCovarianceStamped;

namespace hector_mapping
{

EigenCovariance3 covarianceToEigen(const PoseWithCovariance::_covariance_type &covariance)
{
  return Eigen::Map<const Eigen::Matrix<PoseWithCovariance::_covariance_type::value_type,6,6,Eigen::RowMajor> >(covariance.data()).block<3,3>(0,0).cast<EigenCovariance3::Scalar>();
}

void covarianceToMarker(const PoseWithCovarianceStamped &pose_with_covariance, Marker &marker, bool invert)
{
  marker.header = pose_with_covariance.header;
  covarianceToMarker(pose_with_covariance.pose.pose, covarianceToEigen(pose_with_covariance.pose.covariance), marker, invert);
}

void covarianceToMarker(const tf::StampedTransform &transform, const EigenCovariance3 &covariance, Marker &marker, bool invert)
{
  geometry_msgs::Pose pose;
  tf::poseTFToMsg(transform, pose);
  marker.header.stamp = transform.stamp_;
  marker.header.frame_id = transform.frame_id_;
  covarianceToMarker(pose, covariance, marker, invert);
}

void covarianceToMarker(const geometry_msgs::Pose &pose, const EigenCovariance3 &covariance, Marker &marker, bool invert)
{
  typedef PoseWithCovariance::_covariance_type::value_type Scalar;
  typedef Eigen::Matrix<Scalar,3,1> Vector3;
  typedef Eigen::Matrix<Scalar,3,3> Matrix3x3;

  Eigen::SelfAdjointEigenSolver<Matrix3x3> eigen(covariance);
  Vector3 eigenValues(eigen.eigenvalues());
  Matrix3x3 eigenVectors(eigen.eigenvectors());
  if (eigenVectors.determinant() < 0) eigenVectors.col(0) *= -1.;

  marker.type = Marker::SPHERE;
  if (!invert) {
    marker.scale.x = std::max(sqrt(eigenValues[0]), 0.001);
    marker.scale.y = std::max(sqrt(eigenValues[1]), 0.001);
    marker.scale.z = std::max(sqrt(eigenValues[2]), 0.001);
  } else {
    marker.scale.x = std::max(sqrt(1./eigenValues[0]), 0.001);
    marker.scale.y = std::max(sqrt(1./eigenValues[1]), 0.001);
    marker.scale.z = std::max(sqrt(1./eigenValues[2]), 0.001);
    if (eigenValues[0] == 0.0) marker.scale.x = 0.001;
    if (eigenValues[1] == 0.0) marker.scale.y = 0.001;
    if (eigenValues[2] == 0.0) marker.scale.z = 0.001;
  }

  Eigen::Quaternion<typename Matrix3x3::Scalar> q(eigenVectors);
  marker.pose.position.x = pose.position.x;
  marker.pose.position.y = pose.position.y;
  marker.pose.position.z = pose.position.z;
  marker.pose.orientation.w = q.w();
  marker.pose.orientation.x = q.x();
  marker.pose.orientation.y = q.y();
  marker.pose.orientation.z = q.z();
}

}
