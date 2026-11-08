// Copyright 2025 Lihan Chen
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

#ifndef COSTMAP_TRANSFORMER__COSTMAP_TRANSFORMER_HPP_
#define COSTMAP_TRANSFORMER__COSTMAP_TRANSFORMER_HPP_

#include <memory>
#include <string>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rclcpp/rclcpp.hpp"

namespace costmap_transformer
{

class CostmapTransformerNode : public rclcpp::Node
{
public:
  explicit CostmapTransformerNode(const rclcpp::NodeOptions & options);

private:
  void costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
  nav_msgs::msg::OccupancyGrid::SharedPtr transformCostmap(
    const nav_msgs::msg::OccupancyGrid::SharedPtr msg);

  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_sub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr transformed_costmap_pub_;

  std::string input_topic_;
  std::string output_topic_;
  std::string output_frame_id_;

  double translation_x_;
  double translation_y_;
  double translation_z_;
  double rotation_roll_;
  double rotation_pitch_;
  double rotation_yaw_;
};

}  // namespace costmap_transformer

#endif  // COSTMAP_TRANSFORMER__COSTMAP_TRANSFORMER_HPP_
