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

#ifndef NOISE_FILTER__NOISE_FILTER_HPP_
#define NOISE_FILTER__NOISE_FILTER_HPP_

#include "noise_filter/visibility_control.hpp"
#include "pcl/point_types.h"
#include "pcl/filters/voxel_grid.h"


namespace perception
{
namespace noise_filter
{

class NOISE_FILTER_PUBLIC NoiseFilter
{
public:
  NoiseFilter();
  void setParameters(float leaf_size);
  void downsampleCloud(
    const pcl::PointCloud<pcl::PointXYZ> & pcl_input,
    pcl::PointCloud<pcl::PointXYZ> & pcl_output);
  void removeOutliers(
    const pcl::PointCloud<pcl::PointXYZ> & pcl_input,
    pcl::PointCloud<pcl::PointXYZ> & pcl_output);

private:
  float leaf_size_;
};

}  // namespace noise_filter
}  // namespace perception

#endif  // NOISE_FILTER__NOISE_FILTER_HPP_
