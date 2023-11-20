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

#ifndef NOISE_FILTER__NOISE_FILTER_NODE_HPP_
#define NOISE_FILTER__NOISE_FILTER_NODE_HPP_

#include <memory>
#include "noise_filter/noise_filter.hpp"
#include "pcl_conversions/pcl_conversions.h"
#include "rclcpp/node.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"


namespace perception
{
namespace noise_filter
{

class NOISE_FILTER_PUBLIC NoiseFilterNode : public rclcpp::Node
{
public:
  explicit NoiseFilterNode(const rclcpp::NodeOptions & options);

private:
  void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) const;
  std::unique_ptr<NoiseFilter> noise_filter_ {nullptr};
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
};

}  // namespace noise_filter
}  // namespace perception

#endif  // NOISE_FILTER__NOISE_FILTER_NODE_HPP_
