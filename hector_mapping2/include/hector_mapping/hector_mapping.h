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


#ifndef HECTOR_MAPPING_NODE_H
#define HECTOR_MAPPING_NODE_H

#include <hector_mapping_core/map/types.h>
#include <hector_mapping_core/scan.h>
#include <hector_mapping_core/matcher.h>

#include <ros/ros.h>
#include <nodelet/nodelet.h>

#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/OccupancyGrid.h>


namespace hector_mapping {

class Node : public nodelet::Nodelet
{
public:
  Node();
  virtual ~Node();

  void onInit();

  void scanCallback(const sensor_msgs::LaserScanConstPtr& scan);
  void publishMap();

protected:
  GridMapPtr map_;
  ScanMatcherPtr matcher_;
  Scan scan_;

  nav_msgs::OccupancyGrid map_message_;
  geometry_msgs::PoseWithCovarianceStamped pose_message_;

private:
  ros::Subscriber scan_subscriber_;
  ros::Publisher pose_publisher_;

  ros::Timer map_publish_timer_;
  ros::Publisher map_publisher_;
};

} // namespace hector_mapping

#endif // HECTOR_MAPPING_NODE_H
