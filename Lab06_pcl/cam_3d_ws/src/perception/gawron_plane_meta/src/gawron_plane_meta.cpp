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

#include "gawron_plane_meta/gawron_plane_meta.hpp"

#include <iostream>

namespace perception
{
namespace gawron_plane_meta
{

GawronPlaneMeta::GawronPlaneMeta()
{
}

void GawronPlaneMeta::welcome()
{
  std::cerr << "gawron_plane_meta node start" << std::endl;
}

void GawronPlaneMeta::computeCentroid(pcl::PointCloud<pcl::PointXYZ>::Ptr plane_input,
  pcl::PointXYZ & centroid)
{
  pcl::computeCentroid(*plane_input, centroid);
  std::cerr << "Centroid coordinates: " << centroid.x << " " 
                                        << centroid.y << " "
                                        << centroid.z << std::endl;
}

void GawronPlaneMeta::assignMetadata(gawron_filtering_msgs::msg::Message::SharedPtr metadata_cloud,
  sensor_msgs::msg::PointCloud2::SharedPtr msg,
  pcl::PointXYZ & centroid)
{
  metadata_cloud->header = msg->header;
  metadata_cloud->height = msg->height;
  metadata_cloud->width = msg->width;
  metadata_cloud->length = msg->data.size()/(msg->width*msg->height);
  metadata_cloud->number_of_points = msg->data.size();
  metadata_cloud->center.x = centroid.x;
  metadata_cloud->center.y = centroid.y;
  metadata_cloud->center.z = centroid.z;
}

}   // namespace gawron_plane_meta
}   // namespace perception
