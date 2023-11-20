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

#ifndef SURNAME_PLANE__SURNAME_PLANE_NODE_HPP_
#define SURNAME_PLANE__SURNAME_PLANE_NODE_HPP_

#include <memory>
#include <rclcpp/rclcpp.hpp>

#include "surname_plane/surname_plane.hpp"

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

namespace surname_plane
{
using SurnamePlanePtr = std::unique_ptr<surname_plane::SurnamePlane>;

class SURNAME_PLANE_PUBLIC SurnamePlaneNode : public rclcpp::Node
{
public:
  explicit SurnamePlaneNode(const rclcpp::NodeOptions & options);

private:
  SurnamePlanePtr surname_plane_{nullptr};
  int64_t param_name_{123};
  void planeCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) const;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr plane_pub_;
};
}  // namespace surname_plane

#endif  // SURNAME_PLANE__SURNAME_PLANE_NODE_HPP_
