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

namespace gawron_plane_meta
{

auto custom_qos = rclcpp::QoS(rclcpp::KeepLast(1), rmw_qos_profile_sensor_data);

GawronPlaneMetaNode::GawronPlaneMetaNode(const rclcpp::NodeOptions & options)
:  Node("gawron_plane_meta", options)
{
  plane_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    "/output_cloud_filtered", custom_qos, std::bind(
    &GawronPlaneMetaNode::planeMetaCallback, this,
    std::placeholders::_1));
  msgs_pub_ = this->create_publisher<gawron_filtering_msgs::msg::Message>("output_centroid", custom_qos);
  gawron_plane_meta_ = std::make_unique<gawron_plane_meta::GawronPlaneMeta>();
  param_name_ = this->declare_parameter("param_name", 456);
  gawron_plane_meta_->foo(param_name_);
}

void GawronPlaneMetaNode::planeMetaCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) const
{
  auto metadata_cloud = std::make_unique<gawron_filtering_msgs::msg::Message>();

  pcl::PointCloud<pcl::PointXYZ>::Ptr plane_input(new pcl::PointCloud<pcl::PointXYZ>);

  pcl::fromROSMsg(*msg, *plane_input);

  pcl::PointXYZ centroid;
  pcl::computeCentroid(*plane_input, centroid);

  std::cerr << "Centroid coordinates: " << centroid.x << " " 
                                        << centroid.y << " "
                                        << centroid.z << std::endl;

  metadata_cloud->header = msg->header;
  metadata_cloud->height = msg->height;
  metadata_cloud->width = msg->width;
  metadata_cloud->length = msg->data.size()/(msg->width*msg->height);
  metadata_cloud->number_of_points = msg->data.size();
  metadata_cloud->center.x = centroid.x;
  metadata_cloud->center.y = centroid.y;
  metadata_cloud->center.z = centroid.z;

  msgs_pub_->publish(std::move(metadata_cloud));

  }

}  // namespace gawron_plane_meta

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(gawron_plane_meta::GawronPlaneMetaNode)
