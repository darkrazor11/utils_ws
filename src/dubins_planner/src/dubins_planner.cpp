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

#include "dubins_planner/dubins_planner.hpp"

namespace dubins_planner
{
DubinsPlannerServer::DubinsPlannerServer()
: Node("dubins_planner_node"),
  min_interpolation_threshold_{100.0},
  min_turning_radius_{150.0},
  user_points_{},
  current_pose_{},
  combined_length_{true},
  accumulate_{false},
  start_{false}
{
  RCLCPP_INFO(get_logger(), "Started the Dubins Planner");
  waypoint_marker_pub_ =
    create_publisher<visualization_msgs::msg::MarkerArray>("interpolated_marker", 10);
  user_waypoint_marker_pub_ =
    create_publisher<visualization_msgs::msg::MarkerArray>("user_waypoint_marker", 10);
  pose_array_pub_ = create_publisher<geometry_msgs::msg::PoseArray>("pose_array", 10);
  interpolated_pub_ = create_publisher<geometry_msgs::msg::PoseArray>("interpolated_array", 10);
  optimized_interpolated_pub_ =
    create_publisher<geometry_msgs::msg::PoseArray>("optimized_interpolated_array", 10);
  timer_ = create_wall_timer(50ms, std::bind(&DubinsPlannerServer::timerCallback, this));
  start_sub_ = create_subscription<std_msgs::msg::Bool>(
    "start_optimization", 10,
    std::bind(&DubinsPlannerServer::startCallback, this, std::placeholders::_1));
  waypoint_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
    "goal_pose", 10, std::bind(&DubinsPlannerServer::poseCallback, this, std::placeholders::_1));

  std::vector<std::vector<double>> coordinates = {
    {409061.812874, 1585403.448218}, {409307.367882, 1584416.919671},
    {410292.134242, 1584425.529430}, {411612.324355, 1585345.607631},
    {413225.988130, 1585400.783485}, {414729.291325, 1584409.977204}};
  // std::vector<std::vector<double>> coordinates =
  // {{650575.831256, 1634157.627139}, {651126.219560, 1634394.747982},
  //   {652191.831873, 1634430.645705}, {652691.141747, 1634127.080348},
  //   {653480.666791, 1634375.505676}, {654169.630929, 1634048.831451}};
  // std::vector<std::vector<double>> coordinates =
  // {{243513.867338, 1680348.200203}, {244186.372769, 1680570.818272},
  //   {244625.089300, 1680105.638930}, {244377.772253, 1679455.516564},
  //   {243556.187176, 1679388.879126}, {243112.446033, 1679699.519443},
  //   {243227.247788, 1680433.543509}};
}
void DubinsPlannerServer::startCallback(std_msgs::msg::Bool::SharedPtr msg)
{
  start_ = msg->data;
  if (start_) {
    std::cout << "Starting the optimization" << std::endl;
    accumulate_ = false;
    start_ = false;
    user_points_.clear();
    final_interpolated_array_.clear();

    std::vector<std::vector<double>> coordinates = coordinates_;
    current_pose_.x = (coordinates.front().front() - 500.0);
    current_pose_.y = (coordinates.front().back());
    current_pose_.theta = 0.0;

    for (size_t i = 0; i < coordinates.size(); i++) {
      std::cout << "X,Y : ( " << coordinates[i].front() << ", " << coordinates[i].back() << ")"
                << std::endl;
      geometry_msgs::msg::PointStamped temp_point;
      temp_point.header.frame_id = "map";
      temp_point.point.x = coordinates.at(i).front();
      temp_point.point.y = coordinates.at(i).back();
      user_points_.push_back(temp_point);
    }

    pose2d_array_ = computeOrientation(current_pose_, user_points_);
    pose_array_pub_->publish(getPoseArray(pose2d_array_));

    for (size_t i = 0; i < pose2d_array_.size() - 2; i++) {
      auto dubins_path =
        dubins3PointInterpolation(pose2d_array_[i], pose2d_array_[i + 1], pose2d_array_[i + 2]);
      auto array = interpolateDubinsPath(&dubins_path, 100.0);
      final_interpolated_array_.insert(final_interpolated_array_.end(), array.begin(), array.end());
      pose2d_array_[i + 1] = array.back();

      optimized_interpolated_pub_->publish(getPoseArray(final_interpolated_array_));
    }
    auto last_array =
      dubinsInterpolation(pose2d_array_[pose2d_array_.size() - 2], pose2d_array_.back());
    final_interpolated_array_.insert(
      final_interpolated_array_.end(), last_array.begin(), last_array.end());
    optimized_interpolated_pub_->publish(getPoseArray(final_interpolated_array_));
  }
}

void DubinsPlannerServer::poseCallback(geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  if (!accumulate_) {
    coordinates_.clear();
    accumulate_ = true;
  }
  std::vector<double> point;
  point.push_back(msg->pose.position.x);
  point.push_back(msg->pose.position.y);
  coordinates_.push_back(point);
  std::cout << "Accumulating point and size is " << coordinates_.size() << std::endl;
}

DubinsPath DubinsPlannerServer::dubins3PointInterpolation(
  const geometry_msgs::msg::Pose2D point_1, const geometry_msgs::msg::Pose2D point_2,
  const geometry_msgs::msg::Pose2D point_3)
{
  double start_point[3], goal_point[3], middle_point[3];
  start_point[0] = point_1.x;
  start_point[1] = point_1.y;
  start_point[2] = point_1.theta;
  middle_point[0] = point_2.x;
  middle_point[1] = point_2.y;
  middle_point[2] = 0.0;
  auto theta_start = (point_3.theta - (45 * (M_PI / 180)));
  auto theta_end = (point_3.theta + (45 * (M_PI / 180)));
  goal_point[0] = point_3.x;
  goal_point[1] = point_3.y;
  goal_point[2] = point_3.theta;

  DubinsPath path1, path2;
  // std::vector<DubinsPath> path_vector;
  dubinsPathShortestPath(&path1, start_point, middle_point, min_turning_radius_);
  dubinsPathShortestPath(&path2, middle_point, goal_point, min_turning_radius_);
  double min_length = 0.0;
  if (combined_length_) {
    min_length = dubinsPathLength(&path1) + dubinsPathLength(&path2);
  } else {
    min_length = dubinsPathLength(&path1);
  }
  // std::cout << "The min path length for 0 degree is " << min_length << std::endl;
  for (int angle = (theta_start * (180 / M_PI)); angle < (theta_end * (180 / M_PI)); angle += 5) {
    goal_point[2] = angle * (M_PI / 180);
    for (int i = 0; i < 360; i += 5) {
      // for (int i = 0; i < 360; i++) {
      // for (int i = 0; i < 20; i += 5) {
      middle_point[2] = i * (M_PI / 180);
      DubinsPath temp_path1, temp_path2;
      dubinsPathShortestPath(&temp_path1, start_point, middle_point, min_turning_radius_);
      dubinsPathShortestPath(&temp_path2, middle_point, goal_point, min_turning_radius_);
      double temp_length;
      if (combined_length_) {
        temp_length = dubinsPathLength(&temp_path1) + dubinsPathLength(&temp_path2);
      } else {
        temp_length = dubinsPathLength(&temp_path1);
      }
      std::cout << "The path length of angle : " << (middle_point[2] * (180 / M_PI)) << " is "
                << temp_length << std::endl;
      auto array1 = interpolateDubinsPath(&temp_path1, 100.0);
      auto array2 = interpolateDubinsPath(&temp_path2, 100.0);
      array1.insert(array1.end(), array2.begin(), array2.end());
      interpolated_pub_->publish(getPoseArray(array1));
      if (temp_length < min_length) {
        // std::cout << "the path length is minimum for angle " << (middle_point[2] * (180 / M_PI)) <<
        //   " and length is " << temp_length << std::endl;
        min_length = temp_length;
        path1 = temp_path1;
        path2 = temp_path2;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
      // sleep(std::chrono::milliseconds(500));
    }
  }
  // path_vector.push_back(path1);
  // path_vector.push_back(path2);
  return path1;
}

std::vector<geometry_msgs::msg::Pose2D> DubinsPlannerServer::interpolateDubinsPath(
  DubinsPath * path, double interpolation_distance)
{
  double length = dubinsPathLength(path);
  double inter_point[3];

  std::vector<geometry_msgs::msg::Pose2D> interpolated_2d_poses;
  double dist = interpolation_distance;
  while (dist < length) {
    dubinsPathSample(path, dist, inter_point);
    // dist += 10.0;
    dist += (interpolation_distance);
    geometry_msgs::msg::Pose2D pose;
    pose.x = inter_point[0];
    pose.y = inter_point[1];
    pose.theta = inter_point[2];
    interpolated_2d_poses.push_back(pose);
  }
  dubinsPathSample(path, length, inter_point);
  geometry_msgs::msg::Pose2D pose;
  pose.x = inter_point[0];
  pose.y = inter_point[1];
  pose.theta = inter_point[2];
  interpolated_2d_poses.push_back(pose);
  return interpolated_2d_poses;
}

void DubinsPlannerServer::timerCallback()
{
  pose_array_pub_->publish(getPoseArray(pose2d_array_));
  // interpolated_pub_->publish(getPoseArray(interpolated_array_));
}

geometry_msgs::msg::PoseArray DubinsPlannerServer::getPoseArray(
  std::vector<geometry_msgs::msg::Pose2D> pose2d_array)
{
  geometry_msgs::msg::PoseArray pose_array;
  pose_array.header.frame_id = "map";
  pose_array.header.stamp = this->now();
  for (size_t i = 0; i < pose2d_array.size(); i++) {
    geometry_msgs::msg::Pose pose;
    pose.position.x = pose2d_array[i].x;
    pose.position.y = pose2d_array[i].y;
    tf2::Quaternion quaternion;
    quaternion.setRPY(0.0, 0.0, pose2d_array[i].theta);
    pose.orientation = tf2::toMsg(quaternion);
    pose_array.poses.push_back(pose);
  }
  return pose_array;
}

std::vector<geometry_msgs::msg::Pose2D> DubinsPlannerServer::computeOrientation(
  geometry_msgs::msg::Pose2D start_pose, std::vector<geometry_msgs::msg::PointStamped> waypoints)
{
  std::vector<geometry_msgs::msg::Pose2D> pose2d_array;
  pose2d_array.clear();
  pose2d_array.push_back(current_pose_);
  geometry_msgs::msg::Pose2D pose2d;
  for (unsigned int i = 0; i < waypoints.size() - 1; i++) {
    double dx, dy, yaw;
    dx = waypoints[i + 1].point.x - waypoints[i].point.x;
    dy = waypoints[i + 1].point.y - waypoints[i].point.y;
    yaw = atan2(dy, dx);
    pose2d.x = waypoints[i].point.x;
    pose2d.y = waypoints[i].point.y;
    pose2d.theta = yaw;
    pose2d_array.push_back(pose2d);
  }
  pose2d.x = waypoints.back().point.x;
  pose2d.y = waypoints.back().point.y;
  pose2d.theta = pose2d_array.back().theta;
  pose2d_array.push_back(pose2d);
  return pose2d_array;
}

std::vector<geometry_msgs::msg::Pose2D> DubinsPlannerServer::dubinsInterpolation(
  const geometry_msgs::msg::Pose2D point_1, const geometry_msgs::msg::Pose2D point_2)
{
  double start_point[3], goal_point[3], inter_point[3];
  start_point[0] = point_1.x;
  start_point[1] = point_1.y;
  start_point[2] = point_1.theta;
  goal_point[0] = point_2.x;
  goal_point[1] = point_2.y;
  goal_point[2] = point_2.theta;
  std::vector<geometry_msgs::msg::Pose2D> interpolated_2d_poses;

  auto distance = nav2_util::geometry_utils::euclidean_distance(point_1, point_2);
  if (distance <= min_interpolation_threshold_) {
    interpolated_2d_poses.push_back(point_2);
    return interpolated_2d_poses;
  } else {
    DubinsPath path;
    dubinsPathShortestPath(&path, start_point, goal_point, min_turning_radius_);
    double length = dubinsPathLength(&path);
    std::cout << "The angle is : " << ((point_2.theta) * (180 / M_PI)) << " and the path length is "
              << length << std::endl;
    interpolated_2d_poses.clear();
    // double dist = 10.0;
    double dist = min_interpolation_threshold_;
    while (dist < length) {
      dubinsPathSample(&path, dist, inter_point);
      // dist += 10.0;
      dist += (min_interpolation_threshold_);
      geometry_msgs::msg::Pose2D pose;
      pose.x = inter_point[0];
      pose.y = inter_point[1];
      pose.theta = inter_point[2];
      interpolated_2d_poses.push_back(pose);
    }
    dubinsPathSample(&path, length, inter_point);
    geometry_msgs::msg::Pose2D pose;
    pose.x = inter_point[0];
    pose.y = inter_point[1];
    pose.theta = inter_point[2];
    interpolated_2d_poses.push_back(pose);
  }

  return interpolated_2d_poses;
}

// std::vector<geometry_msgs::msg::Pose2D> DubinsPlannerServer::dubins3PointInterpolation(
//   const geometry_msgs::msg::Pose2D point_1, const geometry_msgs::msg::Pose2D point_2,
//   const geometry_msgs::msg::Pose2D point_3)
// {
//   RCLCPP_INFO(get_logger(), "Started Dubins Interpolation");
//   double start_point[3], goal_point[3], middle_point[3];
//   start_point[0] = point_1.x;
//   start_point[1] = point_1.y;
//   start_point[2] = point_1.theta;
//   middle_point[0] = point_2.x;
//   middle_point[1] = point_2.y;
//   // middle_point[2] = 0.0;
//   middle_point[2] = point_2.theta;
//   goal_point[0] = point_3.x;
//   goal_point[1] = point_3.y;
//   goal_point[2] = point_3.theta;

//   std::vector<geometry_msgs::msg::Pose2D> interpolated_2d_poses;

//   auto distance = nav2_util::geometry_utils::euclidean_distance(point_1, point_2);
//   if (distance <= min_interpolation_threshold_) {
//     interpolated_2d_poses.push_back(point_2);
//     return interpolated_2d_poses;
//   } else {

//     DubinsPath path1, path2;
//     dubinsPathShortestPath(&path1, start_point, middle_point, min_turning_radius_);
//     dubinsPathShortestPath(&path2, middle_point, goal_point, min_turning_radius_);
//     double min_length = dubinsPathLength(&path1) + dubinsPathLength(&path2);
//     // std::cout << "The min length is " << min_length <<std::endl;
//     // for (int i = 1; i < 10; i++) {
//     // for (int i = 1; i < 360; i += 5) {
//     for (int i = 1; i < 360; i++) {
//       // middle_point[2] = 3.14;
//       middle_point[2] = i * M_PI / 180;
//       DubinsPath temp_path1, temp_path2;
//       dubinsPathShortestPath(&temp_path1, start_point, middle_point, min_turning_radius_);
//       dubinsPathShortestPath(&temp_path2, middle_point, goal_point, min_turning_radius_);
//       double temp_length = dubinsPathLength(&temp_path1) + dubinsPathLength(&temp_path2);
//       std::cout << "The length for " << middle_point[2] << " is " << temp_length << std::endl;
//       double inter_point[3];
//       double length = dubinsPathLength(&temp_path1);
//       interpolated_2d_poses.clear();
//       // double dist = 10.0;
//       double dist = min_interpolation_threshold_;
//       while (dist < length) {
//         dubinsPathSample(&path1, dist, inter_point);
//         // dist += 10.0;
//         dist += (min_interpolation_threshold_);
//         geometry_msgs::msg::Pose2D pose;
//         pose.x = inter_point[0];
//         pose.y = inter_point[1];
//         pose.theta = inter_point[2];
//         interpolated_2d_poses.push_back(pose);
//       }
//       dubinsPathSample(&path1, length, inter_point);
//       geometry_msgs::msg::Pose2D pose;
//       pose.x = inter_point[0];
//       pose.y = inter_point[1];
//       pose.theta = inter_point[2];
//       interpolated_2d_poses.push_back(pose);
//       interpolated_array_ = interpolated_2d_poses;
//       interpolated_pub_->publish(getPoseArray(interpolated_array_));
//       if (temp_length < min_length) {
//         min_length = temp_length;
//         path1 = temp_path1;
//         path2 = temp_path2;
//       }
//       sleep(1);
//       std::cout << "size is " << interpolated_array_.size() << std::endl;
//       interpolated_array_.clear();
//       // interpolated_pub_->publish(getPoseArray(interpolated_array_));
//     }
//     double inter_point[3];
//     double length = dubinsPathLength(&path1);

//   }
//   return interpolated_2d_poses;
// }

std::vector<geometry_msgs::msg::PointStamped> DubinsPlannerServer::populateUserPoint()
{
  std::vector<geometry_msgs::msg::PointStamped> user_points;
  geometry_msgs::msg::PointStamped temp_point;
  temp_point.header.frame_id = "map";
  temp_point.point.x = 750.0;
  temp_point.point.y = 750.0;
  user_points.push_back(temp_point);
  temp_point.point.x = 750.0;
  temp_point.point.y = 1500.0;
  user_points.push_back(temp_point);
  temp_point.point.x = 250.0;
  temp_point.point.y = 2000.0;
  user_points.push_back(temp_point);

  return user_points;
}

}  // namespace dubins_planner
