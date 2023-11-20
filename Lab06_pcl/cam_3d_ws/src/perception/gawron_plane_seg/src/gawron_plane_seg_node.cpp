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

#include "gawron_plane_seg/gawron_plane_seg_node.hpp"

namespace perception
{
namespace gawron_plane_seg
{

auto custom_qos = rclcpp::QoS(rclcpp::KeepLast(1), rmw_qos_profile_sensor_data);

GawronPlaneSegNode::GawronPlaneSegNode(const rclcpp::NodeOptions & options)
:  Node("gawron_plane_seg", options)
{
  cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    "/output_cloud_filtered", custom_qos, std::bind(
    &GawronPlaneSegNode::planeCallback, this,
    std::placeholders::_1));
  plane_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("output_plane", custom_qos);
  gawron_plane_seg_ = std::make_unique<GawronPlaneSeg>();
  gawron_plane_seg_->welcome();
}

void GawronPlaneSegNode::planeCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) const
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_input(new pcl::PointCloud<pcl::PointXYZ>);

  pcl::fromROSMsg(*msg, *pcl_input);

  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
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

  // Create a new cloud containing only the inlier points
  pcl::PointCloud<pcl::PointXYZ>::Ptr inlier_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::ExtractIndices<pcl::PointXYZ> extract;

  extract.setInputCloud(pcl_input);
  extract.setIndices(inliers);
  extract.filter(*inlier_cloud);

  sensor_msgs::msg::PointCloud2 plane_msg;

  pcl::toROSMsg(*inlier_cloud, plane_msg);
  plane_msg.header = msg->header;
  plane_pub_->publish(plane_msg);
}

}  // namespace gawron_plane_seg
}  // namespace perception

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(perception::gawron_plane_seg::GawronPlaneSegNode)
