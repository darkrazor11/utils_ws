// Copyright 2024 Anikesh Rajendran
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     https://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef DUBINS_PLANNER__DUBINS_PLANNER_HPP_
#define DUBINS_PLANNER__DUBINS_PLANNER_HPP_

#include <cmath>
#include <limits>
#include <memory>
#include <vector>

#include "dubins_planner/DubinsPath.h"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

using namespace std::chrono_literals;  // NOLINT

namespace dubins_planner
{
class DubinsPlannerServer : public rclcpp::Node
{
public:
  DubinsPlannerServer();

private:
  std::vector<geometry_msgs::msg::Pose2D> computeOrientation(
    const geometry_msgs::msg::Pose2D start_pose,
    const std::vector<geometry_msgs::msg::PointStamped> waypoints);

  std::vector<geometry_msgs::msg::PointStamped> populateUserPoint();
  geometry_msgs::msg::PoseArray getPoseArray(std::vector<geometry_msgs::msg::Pose2D> pose2d_array);

  // std::vector<geometry_msgs::msg::Pose2D> dubins3PointInterpolation(
  //   const geometry_msgs::msg::Pose2D point_1, const geometry_msgs::msg::Pose2D point_2,
  //   const geometry_msgs::msg::Pose2D point_3);
  DubinsPath dubins3PointInterpolation(
    const geometry_msgs::msg::Pose2D point_1, const geometry_msgs::msg::Pose2D point_2,
    const geometry_msgs::msg::Pose2D point_3);
  std::vector<geometry_msgs::msg::Pose2D> dubinsInterpolation(
    const geometry_msgs::msg::Pose2D point_1, const geometry_msgs::msg::Pose2D point_2);
  std::vector<geometry_msgs::msg::Pose2D> interpolateDubinsPath(
    DubinsPath * path, double interpolation_distance);

  void timerCallback();
  void startCallback(std_msgs::msg::Bool::SharedPtr msg);
  void poseCallback(geometry_msgs::msg::PoseStamped::SharedPtr msg);

  double min_turning_radius_{0.0}, min_interpolation_threshold_{0.0};
  geometry_msgs::msg::Pose2D current_pose_;
  std::vector<geometry_msgs::msg::PointStamped> user_points_;
  std::vector<geometry_msgs::msg::Pose2D> pose2d_array_, interpolated_array_,
    final_interpolated_array_;
  geometry_msgs::msg::PoseArray pose_array_;
  std::vector<std::vector<double>> coordinates_;

  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr waypoint_marker_pub_,
    user_waypoint_marker_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr pose_array_pub_, interpolated_pub_,
    optimized_interpolated_pub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr start_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr waypoint_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  bool combined_length_, accumulate_, start_;
};
}  // namespace dubins_planner

#endif  // DUBINS_PLANNER__DUBINS_PLANNER_HPP_
