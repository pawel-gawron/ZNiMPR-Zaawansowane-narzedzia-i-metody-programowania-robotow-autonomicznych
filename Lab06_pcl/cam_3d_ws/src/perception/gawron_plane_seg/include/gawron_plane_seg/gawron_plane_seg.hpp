// Copyright 2023 Pawel_Gawron
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef GAWRON_PLANE_SEG__GAWRON_PLANE_SEG_HPP_
#define GAWRON_PLANE_SEG__GAWRON_PLANE_SEG_HPP_

#include <cstdint>

#include "gawron_plane_seg/visibility_control.hpp"
#include "pcl/point_types.h"
#include "pcl/filters/voxel_grid.h"

#include "pcl_conversions/pcl_conversions.h"
#include "rclcpp/node.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>


namespace perception
{
namespace gawron_plane_seg
{

class GAWRON_PLANE_SEG_PUBLIC GawronPlaneSeg
{
public:
  GawronPlaneSeg();
  void welcome();
  void segmentation(const pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_input,
  pcl::ModelCoefficients::Ptr & coefficients,
  pcl::PointIndices::Ptr & inliers);
  void extract(const pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_input,
  pcl::ModelCoefficients::Ptr & coefficients,
  pcl::PointIndices::Ptr & inliers);
  void extract(const pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_input,
  pcl::PointIndices::Ptr & inliers,
  sensor_msgs::msg::PointCloud2 & plane_msg,
  const sensor_msgs::msg::PointCloud2::SharedPtr & msg);
};

}  // namespace gawron_plane_seg
}  // namespace perception

#endif  // GAWRON_PLANE_SEG__GAWRON_PLANE_SEG_HPP_
