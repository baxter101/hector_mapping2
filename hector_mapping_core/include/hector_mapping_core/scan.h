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


#ifndef HECTOR_MAPPING_SCAN_H
#define HECTOR_MAPPING_SCAN_H

#include <hector_mapping_core/types.h>
#include <hector_mapping_core/parameters.h>
#include <hector_mapping_core/internal/macros.h>

#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>

#include <limits>

#include <ros/publisher.h>

// forward declarations
namespace laser_geometry { class LaserProjection; }
namespace tf { class Transformer; }

namespace hector_mapping {

class ScanParameters {
  PARAMETER(ScanParameters, double, min_distance);
  PARAMETER(ScanParameters, double, max_distance);
  PARAMETER(ScanParameters, int, channel_options);
  PARAMETER(ScanParameters, double, min_z);
  PARAMETER(ScanParameters, double, max_z);

public:
  ScanParameters();
};

class Scan
{
public:
  typedef std::vector<Point>::const_iterator iterator;

  Scan(const Parameters& params = Parameters());
  virtual ~Scan();

  Scan& setTransformer(tf::Transformer& tf, const std::string &target_frame);
  ros::Publisher advertisePointCloud(ros::NodeHandle& nh, std::string topic = std::string());

  const std_msgs::Header &getHeader() const { return header_; }
  const ros::Time &getStamp() const { return header_.stamp; }

  const tf::StampedTransform& getStampedTransform() const { return transform_; }
  const tf::Transform& getTransform() const { return transform_; }

  bool valid() const;
  void clear();

  std::size_t size() const { return points_.size(); }
  iterator begin() const { return points_.begin(); }
  iterator end() const   { return points_.end(); }

  Scan& operator=(const sensor_msgs::LaserScanConstPtr& scan);
  Scan& operator=(const sensor_msgs::PointCloud2& points);

private:
  ScanParameters scan_params_;

  tf::Transformer *tf_;
  std::string target_frame_;

  std_msgs::Header header_;
  std::vector<Point> points_;
  tf::StampedTransform transform_;

  boost::shared_ptr<laser_geometry::LaserProjection> laser_projection_;
  ros::Publisher scan_cloud_publisher_;
};

} // namespace hector_mapping

#endif // HECTOR_MAPPING_SCAN_H
