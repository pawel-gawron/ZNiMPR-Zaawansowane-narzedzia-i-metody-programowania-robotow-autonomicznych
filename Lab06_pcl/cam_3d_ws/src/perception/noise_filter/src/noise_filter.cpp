// Copyright 2022 Amadeusz Szymko
// Perception for Physical Interaction Laboratory at Poznan University of Technology
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "noise_filter/noise_filter.hpp"


namespace perception
{
namespace noise_filter
{

NoiseFilter::NoiseFilter(){}

void NoiseFilter::downsampleCloud(
  const pcl::PointCloud<pcl::PointXYZ> & pcl_input,
  pcl::PointCloud<pcl::PointXYZ> & pcl_output)
{
  pcl::VoxelGrid<pcl::PointXYZ> vg;
  vg.setInputCloud(pcl_input.makeShared());
  vg.setLeafSize(leaf_size_, leaf_size_, leaf_size_);
  vg.filter(pcl_output);
}

void NoiseFilter::setParameters(float leaf_size)
{
  leaf_size_ = leaf_size;
}


}  // namespace noise_filter
}  // namespace perception
