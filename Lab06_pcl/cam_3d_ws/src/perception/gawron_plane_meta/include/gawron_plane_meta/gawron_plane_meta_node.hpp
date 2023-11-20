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

#ifndef GAWRON_PLANE_META__GAWRON_PLANE_META_NODE_HPP_
#define GAWRON_PLANE_META__GAWRON_PLANE_META_NODE_HPP_

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include "sensor_msgs/msg/point_cloud2.hpp"

#include "pcl_conversions/pcl_conversions.h"
#include "rclcpp/node.hpp"
#include "rclcpp/rclcpp.hpp"
#include "gawron_plane_meta/gawron_plane_meta.hpp"

#include <pcl/point_cloud.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/common/centroid.h>

#include "gawron_filtering_msgs/msg/message.hpp"

namespace perception
{
namespace gawron_plane_meta
{
using GawronPlaneMetaPtr = std::unique_ptr<GawronPlaneMeta>;

class GAWRON_PLANE_META_PUBLIC GawronPlaneMetaNode : public rclcpp::Node
{
public:
  explicit GawronPlaneMetaNode(const rclcpp::NodeOptions & options);

private:
  GawronPlaneMetaPtr gawron_plane_meta_{nullptr};
  int64_t param_name_{123};
  void planeMetaCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) const;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr plane_sub_;
  rclcpp::Publisher<gawron_filtering_msgs::msg::Message>::SharedPtr msgs_pub_;
};
}  // namespace gawron_plane_meta
}  // namespace perception

#endif  // GAWRON_PLANE_META__GAWRON_PLANE_META_NODE_HPP_
