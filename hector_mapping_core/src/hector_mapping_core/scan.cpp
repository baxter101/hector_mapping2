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

#include <hector_mapping_core/scan.h>
#include <laser_geometry/laser_geometry.h>

#include <boost/function.hpp>

namespace hector_mapping {

ScanParameters::ScanParameters()
  : min_distance_(0.0)
  , max_distance_(-1.0)
  , channel_options_(0)
  , min_z_(-std::numeric_limits<double>::quiet_NaN())
  , max_z_( std::numeric_limits<double>::quiet_NaN())
{}

Scan::Scan(const Parameters& _params)
  : tf_(0)
{
  _params.add("scan", scan_params_);
  laser_projection_.reset(new laser_geometry::LaserProjection);
}

Scan::~Scan()
{
}

Scan &Scan::setTransformer(tf::Transformer &tf, const std::string &target_frame)
{
  tf_ = &tf;
  target_frame_ = target_frame;
}

ros::Publisher Scan::advertisePointCloud(ros::NodeHandle& nh, std::string topic)
{
  if (topic.empty()) topic = "scan_cloud";
  scan_cloud_publisher_ = nh.advertise<sensor_msgs::PointCloud2>(topic, 10);
  return scan_cloud_publisher_;
}

bool Scan::valid() const
{
  return points_.size() > 0;
}

void Scan::clear()
{
  points_.clear();
}

Scan& Scan::operator=(const sensor_msgs::LaserScanConstPtr& scan)
{
  sensor_msgs::PointCloud2 cloud;

  if (tf_) {
    try {
      laser_projection_->transformLaserScanToPointCloud(target_frame_, *scan, cloud, *tf_, scan_params_.max_distance(), scan_params_.channel_options());
    } catch(tf::TransformException& e) {
      ROS_WARN("%s", e.what());
      clear();
      return *this;
    }
  } else {
    laser_projection_->projectLaser(*scan, cloud, scan_params_.max_distance(), scan_params_.channel_options());
  }

  return *this = cloud;
}

namespace internal {
  template <typename T> struct GetPoint {
    typedef T result_type;
    T operator()(const uint8_t *data, const sensor_msgs::PointField& field) {
      return *reinterpret_cast<const T *>(data + field.offset);
    }
  };

  typedef boost::function<float_t(const uint8_t *data)> GetPointFunc;
  GetPointFunc getPointFunc(const sensor_msgs::PointField& field) {
    switch(field.datatype) {
      case sensor_msgs::PointField::FLOAT32:
        return boost::bind(GetPoint<float>(), _1, field);
      case sensor_msgs::PointField::FLOAT64:
        return boost::bind(GetPoint<double>(), _1, field);
      default:
        ROS_ERROR("Illegal field type %u for point cloud field %s.", field.datatype, field.name.c_str());
        return GetPointFunc();
    }
  }
}

Scan& Scan::operator=(const sensor_msgs::PointCloud2& cloud)
{
  sensor_msgs::PointCloud2Ptr filtered_cloud;

  if (scan_cloud_publisher_) {
    filtered_cloud.reset(new sensor_msgs::PointCloud2);
    filtered_cloud->header = cloud.header;
    filtered_cloud->height = 1;
    filtered_cloud->fields = cloud.fields;
    filtered_cloud->is_bigendian = cloud.is_bigendian;
    filtered_cloud->point_step = cloud.point_step;
    filtered_cloud->row_step = 0;
    filtered_cloud->is_dense = true;
  }

  // save header
  header_ = cloud.header;

  // search x, y and z fields
  std::map<std::string,sensor_msgs::PointCloud2::_fields_type::const_iterator> fields;
  for(sensor_msgs::PointCloud2::_fields_type::const_iterator field = cloud.fields.begin(); field != cloud.fields.end(); ++field) {
    fields[field->name] = field;
  }

  if (!fields.count("x") || !fields.count("y") || !fields.count("z")) {
    ROS_ERROR("Input point cloud does not have x, y and z fields!");
    clear();
    return *this;
  }
  internal::GetPointFunc x = internal::getPointFunc(*fields["x"]);
  internal::GetPointFunc y = internal::getPointFunc(*fields["y"]);
  internal::GetPointFunc z = internal::getPointFunc(*fields["z"]);

  // resize scan
  clear();
  points_.reserve(cloud.width * cloud.height);

  sensor_msgs::PointCloud2::_data_type::const_iterator it = cloud.data.begin();
  for(std::size_t i = 0;
      it < cloud.data.end();
      ++i, it += cloud.point_step) {
    const uint8_t *data = &(*it);
    Point point(x(data), y(data), z(data));
    double distance = point.norm();
    if (distance >= scan_params_.max_distance()) continue;
    if (distance <  scan_params_.min_distance()) continue;
    if (point.z() < scan_params_.min_z()) continue;
    if (point.z() > scan_params_.max_z()) continue;
    points_.push_back(point);
    if (filtered_cloud) filtered_cloud->data.insert(filtered_cloud->data.end(), it, it + cloud.point_step);
  }

  if (filtered_cloud) {
    filtered_cloud->width = points_.size();
    scan_cloud_publisher_.publish(filtered_cloud);
  }

  return *this;
}

} // namespace hector_mapping

