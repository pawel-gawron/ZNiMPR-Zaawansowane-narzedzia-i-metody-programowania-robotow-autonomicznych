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

#include "gawron_plane_meta/gawron_plane_meta_node.hpp"

namespace perception
{
namespace gawron_plane_meta
{

auto custom_qos = rclcpp::QoS(rclcpp::KeepLast(1), rmw_qos_profile_sensor_data);

GawronPlaneMetaNode::GawronPlaneMetaNode(const rclcpp::NodeOptions & options)
:  Node("gawron_plane_meta", options)
{
  plane_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    "input_cloud", custom_qos, std::bind(
    &GawronPlaneMetaNode::planeMetaCallback, this,
    std::placeholders::_1));
  msgs_pub_ = this->create_publisher<gawron_filtering_msgs::msg::Message>("metadata_cloud", custom_qos);
  gawron_plane_meta_ = std::make_unique<GawronPlaneMeta>();
  gawron_plane_meta_->welcome();
}

void GawronPlaneMetaNode::planeMetaCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) const
{
  auto metadata_cloud = std::make_shared<gawron_filtering_msgs::msg::Message>();

  pcl::PointCloud<pcl::PointXYZ>::Ptr plane_input(new pcl::PointCloud<pcl::PointXYZ>);

  pcl::fromROSMsg(*msg, *plane_input);

  pcl::PointXYZ centroid;
  
  gawron_plane_meta_->computeCentroid(plane_input, centroid);

  gawron_plane_meta_->assignMetadata(metadata_cloud, msg, centroid);

  msgs_pub_->publish(*metadata_cloud);

  }

}  // namespace gawron_plane_meta
}  // namespace perception

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(perception::gawron_plane_meta::GawronPlaneMetaNode)
