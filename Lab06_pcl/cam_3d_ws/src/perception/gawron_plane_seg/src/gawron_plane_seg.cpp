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

#include "gawron_plane_seg/gawron_plane_seg.hpp"

#include <iostream>

namespace perception
{
namespace gawron_plane_seg
{

GawronPlaneSeg::GawronPlaneSeg()
{
}

void GawronPlaneSeg::welcome()
{
  std::cerr << "gawron_plane_seg node start" << std::endl;
}

void GawronPlaneSeg::segmentation(const pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_input,
  pcl::ModelCoefficients::Ptr & coefficients,
  pcl::PointIndices::Ptr & inliers)
{
  // Create the segmentation object
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  // Optional
  seg.setOptimizeCoefficients(true);
  // Mandatory
  seg.setModelType(pcl::SACMODEL_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setDistanceThreshold(0.005);

  seg.setInputCloud(pcl_input);
  seg.segment(*inliers, *coefficients);

  std::cerr << "Model coefficients: " << coefficients->values[0] << " " 
                                      << coefficients->values[1] << " "
                                      << coefficients->values[2] << " " 
                                      << coefficients->values[3] << std::endl;
}

void GawronPlaneSeg::extract(const pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_input,
  pcl::PointIndices::Ptr & inliers,
  sensor_msgs::msg::PointCloud2 & plane_msg,
  const sensor_msgs::msg::PointCloud2::SharedPtr & msg)
{
  // Create a new cloud containing only the inlier points
  pcl::PointCloud<pcl::PointXYZ>::Ptr inlier_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::ExtractIndices<pcl::PointXYZ> extract;

  extract.setInputCloud(pcl_input);
  extract.setIndices(inliers);
  extract.filter(*inlier_cloud);

  pcl::toROSMsg(*inlier_cloud, plane_msg);
  plane_msg.header = msg->header;
}

}  // namespace gawron_plane_seg
}  // namespace perception
