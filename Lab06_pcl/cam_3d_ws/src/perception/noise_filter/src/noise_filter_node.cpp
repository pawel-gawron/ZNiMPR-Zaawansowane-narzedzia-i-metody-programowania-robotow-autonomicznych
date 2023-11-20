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

#include "noise_filter/noise_filter_node.hpp"


using namespace std::chrono_literals;

namespace perception
{
namespace noise_filter
{
auto custom_qos = rclcpp::QoS(rclcpp::KeepLast(1), rmw_qos_profile_sensor_data);

NoiseFilterNode::NoiseFilterNode(const rclcpp::NodeOptions & options)
: Node("noise_filter", options)
{
  cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    "input_cloud", custom_qos, std::bind(
      &NoiseFilterNode::cloudCallback, this,
      std::placeholders::_1));
  cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("output_cloud", custom_qos);
  auto leaf_size = this->declare_parameter("leaf_size", 0.001);
  noise_filter_ = std::make_unique<NoiseFilter>();
  noise_filter_->setParameters(leaf_size);
}

void NoiseFilterNode::cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) const
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_input(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_downsampled(new pcl::PointCloud<pcl::PointXYZ>);
  sensor_msgs::msg::PointCloud2 filtered_msg;

  pcl::fromROSMsg(*msg, *pcl_input);
  noise_filter_->downsampleCloud(*pcl_input, *pcl_downsampled);
  // noise_filter_->removeOutliers(*pcl_downsampled, *pcl_filtered);

  pcl::toROSMsg(*pcl_downsampled, filtered_msg);
  filtered_msg.header = msg->header;
  cloud_pub_->publish(filtered_msg);
}


}  // namespace noise_filter
}  // namespace perception

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(perception::noise_filter::NoiseFilterNode)
