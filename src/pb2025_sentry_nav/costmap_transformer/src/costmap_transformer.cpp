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

#include "costmap_transformer/costmap_transformer.hpp"

#include <cmath>
#include <vector>

namespace costmap_transformer
{

CostmapTransformerNode::CostmapTransformerNode(const rclcpp::NodeOptions & options)
: Node("costmap_transformer", options)
{
  this->declare_parameter("input_topic", "global_costmap/costmap");
  this->declare_parameter("output_topic", "global_costmap/costmap_transformed");
  this->declare_parameter("output_frame_id", "map");
  this->declare_parameter("translation_x", 0.0);
  this->declare_parameter("translation_y", 0.0);
  this->declare_parameter("translation_z", 0.0);
  this->declare_parameter("rotation_roll", 0.0);
  this->declare_parameter("rotation_pitch", 0.0);
  this->declare_parameter("rotation_yaw", 0.0);

  this->get_parameter("input_topic", input_topic_);
  this->get_parameter("output_topic", output_topic_);
  this->get_parameter("output_frame_id", output_frame_id_);
  this->get_parameter("translation_x", translation_x_);
  this->get_parameter("translation_y", translation_y_);
  this->get_parameter("translation_z", translation_z_);
  this->get_parameter("rotation_roll", rotation_roll_);
  this->get_parameter("rotation_pitch", rotation_pitch_);
  this->get_parameter("rotation_yaw", rotation_yaw_);

  RCLCPP_INFO(
    this->get_logger(),
    "CostmapTransformer initialized with translation [%.3f, %.3f, %.3f] and rotation [%.3f, %.3f, "
    "%.3f]",
    translation_x_, translation_y_, translation_z_, rotation_roll_, rotation_pitch_,
    rotation_yaw_);

  costmap_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
    input_topic_, 10,
    std::bind(&CostmapTransformerNode::costmapCallback, this, std::placeholders::_1));

  transformed_costmap_pub_ =
    this->create_publisher<nav_msgs::msg::OccupancyGrid>(output_topic_, 10);
}

void CostmapTransformerNode::costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
  auto transformed = transformCostmap(msg);
  if (transformed) {
    transformed_costmap_pub_->publish(*transformed);
  }
}

nav_msgs::msg::OccupancyGrid::SharedPtr CostmapTransformerNode::transformCostmap(
  const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
  auto result = std::make_shared<nav_msgs::msg::OccupancyGrid>();
  result->header = msg->header;
  result->header.frame_id = output_frame_id_;
  result->info = msg->info;

  // Precompute rotation matrix (ZYX order: yaw, pitch, roll)
  const double cr = std::cos(rotation_roll_);
  const double sr = std::sin(rotation_roll_);
  const double cp = std::cos(rotation_pitch_);
  const double sp = std::sin(rotation_pitch_);
  const double cy = std::cos(rotation_yaw_);
  const double sy = std::sin(rotation_yaw_);

  // Rotation matrix R = Rz(yaw) * Ry(pitch) * Rx(roll)
  const double r00 = cy * cp;
  const double r01 = cy * sp * sr - sy * cr;
  const double r02 = cy * sp * cr + sy * sr;
  const double r10 = sy * cp;
  const double r11 = sy * sp * sr + cy * cr;
  const double r12 = sy * sp * cr - cy * sr;
  const double r20 = -sp;
  const double r21 = cp * sr;
  const double r22 = cp * cr;

  // Inverse rotation (transpose of R)
  const double ir00 = r00;
  const double ir01 = r10;
  const double ir02 = r20;
  const double ir10 = r01;
  const double ir11 = r11;
  const double ir12 = r21;
  const double ir20 = r02;
  const double ir21 = r12;
  const double ir22 = r22;

  // Original map origin
  const double orig_x = msg->info.origin.position.x;
  const double orig_y = msg->info.origin.position.y;
  const double orig_z = msg->info.origin.position.z;
  const double res = msg->info.resolution;
  const int width = static_cast<int>(msg->info.width);
  const int height = static_cast<int>(msg->info.height);

  // Compute the four corners of the original map in world coordinates
  std::vector<std::pair<double, double>> corners;
  corners.reserve(4);
  corners.emplace_back(orig_x, orig_y);
  corners.emplace_back(orig_x + width * res, orig_y);
  corners.emplace_back(orig_x + width * res, orig_y + height * res);
  corners.emplace_back(orig_x, orig_y + height * res);

  // Transform corners to find new bounding box
  double min_tx = std::numeric_limits<double>::max();
  double min_ty = std::numeric_limits<double>::max();
  double max_tx = std::numeric_limits<double>::lowest();
  double max_ty = std::numeric_limits<double>::lowest();

  for (const auto & corner : corners) {
    // new_point = R * old_point + t
    double wx = r00 * corner.first + r01 * corner.second + r02 * orig_z + translation_x_;
    double wy = r10 * corner.first + r11 * corner.second + r12 * orig_z + translation_y_;

    min_tx = std::min(min_tx, wx);
    min_ty = std::min(min_ty, wy);
    max_tx = std::max(max_tx, wx);
    max_ty = std::max(max_ty, wy);
  }

  // Compute new map dimensions (aligned to resolution grid)
  const int new_width =
    static_cast<int>(std::ceil((max_tx - min_tx) / res)) + 1;
  const int new_height =
    static_cast<int>(std::ceil((max_ty - min_ty) / res)) + 1;

  result->info.width = static_cast<uint32_t>(new_width);
  result->info.height = static_cast<uint32_t>(new_height);
  result->info.origin.position.x = min_tx;
  result->info.origin.position.y = min_ty;
  result->info.origin.position.z = orig_z + translation_z_;
  result->info.origin.orientation.x = 0.0;
  result->info.origin.orientation.y = 0.0;
  result->info.origin.orientation.z = 0.0;
  result->info.origin.orientation.w = 1.0;

  // Initialize with unknown (-1)
  result->data.resize(new_width * new_height, -1);

  // For each cell in the new map, find the corresponding cell in the original map
  for (int ny = 0; ny < new_height; ++ny) {
    for (int nx = 0; nx < new_width; ++nx) {
      // World coordinates in the new (transformed) map
      double wx = min_tx + (nx + 0.5) * res;
      double wy = min_ty + (ny + 0.5) * res;
      double wz = orig_z + translation_z_;

      // Inverse transform: old_point = R^T * (new_point - t)
      double dx = wx - translation_x_;
      double dy = wy - translation_y_;
      double dz = wz - translation_z_;

      double ox = ir00 * dx + ir01 * dy + ir02 * dz;
      double oy = ir10 * dx + ir11 * dy + ir12 * dz;
      // double oz = ir20 * dx + ir21 * dy + ir22 * dz;

      // Map to original grid coordinates
      int ox_idx = static_cast<int>((ox - orig_x) / res);
      int oy_idx = static_cast<int>((oy - orig_y) / res);

      if (ox_idx >= 0 && ox_idx < width && oy_idx >= 0 && oy_idx < height) {
        int orig_index = oy_idx * width + ox_idx;
        int new_index = ny * new_width + nx;
        result->data[new_index] = msg->data[orig_index];
      }
    }
  }

  RCLCPP_INFO(
    this->get_logger(), "Transformed costmap: %dx%d -> %dx%d", width, height, new_width,
    new_height);

  return result;
}

}  // namespace costmap_transformer

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(costmap_transformer::CostmapTransformerNode)
