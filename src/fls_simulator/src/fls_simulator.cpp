#include <vector>
#include <memory>
#include <random>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/polygon_stamped.hpp"
#include "geometry_msgs/msg/point32.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "nav2_util/robot_utils.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "obstacle_pose_interface/msg/obstacles_in_fov_of_sensor.hpp"
#include "obstacle_pose_interface/msg/obstacle_range_bearing.hpp"
#include <GeographicLib/UTMUPS.hpp>

using namespace std::chrono_literals;

class FlsSimulator : public rclcpp::Node
{
public:
  FlsSimulator()
  : Node("fls_simulator")
  {
    int zone;
    bool northp;
    double lat, lon, x_utm, y_utm;
    lat = 56.7122416123972;
    lon = 3.50337795561836;
    GeographicLib::UTMUPS::Forward(lat, lon, zone, northp, x_utm, y_utm);

    tf_buffer_ =
      std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ =
      std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    odom_sub_ =
      this->create_subscription<nav_msgs::msg::Odometry>(
      "/odometry", 10,
      std::bind(&FlsSimulator::odomCB, this, std::placeholders::_1));
    fls_window_pub_ = create_publisher<geometry_msgs::msg::PolygonStamped>("/fls_window", 10);
    marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("/obstacle_marker", 10);
    detected_marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      "/detected_marker", 10);
    fls_fov_pub_ = create_publisher<obstacle_pose_interface::msg::ObstaclesInFOVOfSensor>(
      "/heauv/fls/obstacles_in_FOV", rclcpp::SystemDefaultsQoS());


    // std::cout << "x " << x_utm << " y " << y_utm << std::endl;


    for (int i = 0; i < 10000; i++) {
      geometry_msgs::msg::Point obstacle = generateRandomObstacle();
      generated_obstacles.push_back(obstacle);
    }

  }

private:
  bool transformPointToTargetFrame(
    const geometry_msgs::msg::PointStamped & point_in, geometry_msgs::msg::PointStamped & point_out,
    const std::string & input_frame, const std::string & target_frame)
  {
    try {
      geometry_msgs::msg::TransformStamped transform_stamped =
        tf_buffer_->lookupTransform(
        target_frame, input_frame,
        tf2::TimePointZero);

      tf2::doTransform(point_in, point_out, transform_stamped);

      return true;
    } catch (tf2::TransformException & ex) {
      RCLCPP_WARN(get_logger(), "Failed to transform footprint: %s", ex.what());
      return false;
    }
  }
  void odomCB(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    marker_pub_->publish(publishObstacleMarker(generated_obstacles, false));

    double x = msg->pose.pose.position.x;
    double y = msg->pose.pose.position.y;

    tf2::Quaternion q(
      msg->pose.pose.orientation.x,
      msg->pose.pose.orientation.y,
      msg->pose.pose.orientation.z,
      msg->pose.pose.orientation.w);
    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

    // Define the polygon window in front of the robot
    double distance_ahead = 10.0;     // Distance in front of the robot (adjust as needed)
    double width = 10.5;              // Width of the window (adjust as needed)

    geometry_msgs::msg::PolygonStamped polygon_msg;
    polygon_msg.header.frame_id = "CAIR";
    polygon_msg.header.stamp = this->get_clock()->now();

    // Create points for the triangular polygon
    geometry_msgs::msg::Point32 p1, p2, p3;

// Front vertex of the triangle (200 meters ahead of the robot)
    p1.x = x;
    p1.y = y;
    // p1.x = x + 200.0 * cos(yaw);
    // p1.y = y + 200.0 * sin(yaw);

// Left vertex of the triangle (-45 degrees relative to robot's orientation)
    p2.x = x + 200.0 * cos(yaw - M_PI / 4);
    p2.y = y + 200.0 * sin(yaw - M_PI / 4);

// Right vertex of the triangle (45 degrees relative to robot's orientation)
    p3.x = x + 200.0 * cos(yaw + M_PI / 4);
    p3.y = y + 200.0 * sin(yaw + M_PI / 4);

// Push points into the polygon message
    polygon_msg.polygon.points.push_back(p1);
    polygon_msg.polygon.points.push_back(p2);
    polygon_msg.polygon.points.push_back(p3);

    fls_window_pub_->publish(polygon_msg);
    detected_obstacles.clear();
    for (auto obstacle : generated_obstacles) {
      geometry_msgs::msg::PointStamped input_pose, transformed_pose;
      input_pose.header.frame_id = "obstacle";
      input_pose.point = obstacle;

      // std::cout << "before x: " << input_pose.point.x << " y: " <<
      //   input_pose.point.y << std::endl;
      // nav2_util::transformPoseInTargetFrame(input_pose, transformed_pose, *tf_, "CAIR", 1.0);

      transformPointToTargetFrame(
        input_pose, transformed_pose,
        "obstacle", "CAIR");

      // std::cout << "x: " << transformed_pose.point.x << " y: " <<
      //   transformed_pose.point.y << std::endl;

      if (isObstacleInsideRange(transformed_pose.point, polygon_msg.polygon)) {
        geometry_msgs::msg::PointStamped input, output;
        input = transformed_pose;
        transformPointToTargetFrame(
          input, output, "CAIR", "sonarfront_link");
        // output.point.y = -output.point.y;

        detected_obstacles.push_back(output.point);

        // Publish sensor data if obstacle is detected
        // publishSensorData(obstacle, polygon_msg);
        // std::cout << "Detected obstacles are " << detected_obstacles.size() << std::endl;
        detected_marker_pub_->publish(publishObstacleMarker(detected_obstacles, true));
      }

      obstacle_pose_interface::msg::ObstacleRangeBearing temp_obstacle;
      obstacle_pose_interface::msg::ObstaclesInFOVOfSensor obstacles;
      for (auto obstacle : detected_obstacles) {
        double range, bearing;
        convertToRangeBearing(obstacle.x, obstacle.y, yaw, range, bearing);
        temp_obstacle.range = range;
        // temp_obstacle.bearing = bearing;
        temp_obstacle.bearing = bearing * (180 / M_PI);
        temp_obstacle.header.frame_id = "sonarfront_link";
        temp_obstacle.header.stamp = now();
        temp_obstacle.bearing_extent = 0.10;
        temp_obstacle.range_extent = 0.5;
        obstacles.obstacles_list.push_back(temp_obstacle);
      }
      fls_fov_pub_->publish(obstacles);
    }
    // sleep(1);
    // detected_marker_pub_->publish
  }
  void convertToRangeBearing(double x, double y, double yaw, double & range, double & bearing)
  {
    range = std::sqrt(x * x + y * y);
    double raw_bearing = std::atan2(y, x);
    // bearing = raw_bearing - yaw;
    bearing = raw_bearing;
    while (bearing > M_PI) {
      bearing -= 2 * M_PI;
    }
    while (bearing < M_PI) {
      bearing += 2 * M_PI;
    }
  }
  bool isObstacleInsideRange(
    const geometry_msgs::msg::Point & obstacle,
    const geometry_msgs::msg::Polygon & polygon)
  {
    bool inside = true;
    if (polygon.points.size() != 0) {
      int num_vertices = polygon.points.size();
      inside = false;

      for (int i = 0, j = num_vertices - 1; i < num_vertices; j = i++) {
        if ((polygon.points[i].y > obstacle.y) !=
          (polygon.points[j].y > obstacle.y) &&
          obstacle.x <
          (polygon.points[j].x - polygon.points[i].x) *
          (obstacle.y - polygon.points[i].y) /
          (polygon.points[j].y - polygon.points[i].y) + polygon.points[i].x)
        {
          inside = !inside;
        }

        if ((polygon.points[i].y == obstacle.y) &&
          (obstacle.x >=
          std::min(
            polygon.points[i].x,
            polygon.points[j].x)) &&
          (obstacle.x <=
          std::max(polygon.points[i].x, polygon.points[j].x)))
        {
          inside = true;
        }
      }
    }
    return inside;
  }

  geometry_msgs::msg::Point generateRandomObstacle()
  {
    double spread_x{3000.0}, spread_y{3000.0};
    static std::default_random_engine generator;
    std::uniform_real_distribution<double> distribution_x(-spread_x, spread_x);
    std::uniform_real_distribution<double> distribution_y(-spread_y, spread_y);
    geometry_msgs::msg::Point obstacle;
    obstacle.x = distribution_x(generator);
    obstacle.y = distribution_y(generator);
    obstacle.z = 0.0;
    return obstacle;
  }

  visualization_msgs::msg::MarkerArray publishObstacleMarker(
    const std::vector<geometry_msgs::msg::Point> & obstacles, bool red)
  {
    visualization_msgs::msg::MarkerArray marker_array;
    int id = 0;
    for (const auto obstacle: obstacles) {
      visualization_msgs::msg::Marker marker;
      if (!red) {
        marker.header.frame_id = "obstacle";
      } else {
        marker.header.frame_id = "sonarfront_link";
      }
      marker.header.stamp = now();
      marker.ns = "obstacles";
      marker.id = id++;
      marker.type = marker.SPHERE;
      marker.action = marker.ADD;

      marker.pose.position = obstacle;
      marker.pose.orientation.w = 1.0;

      marker.scale.x = 5.0;
      marker.scale.y = 5.0;
      marker.scale.z = 2.0;

      marker.color.a = 1.0;
      if (red) {
        marker.color.r = 255.0;
        marker.color.g = 0.10;
        marker.color.b = 0.0;
      } else {
        marker.color.r = 100.0;
        marker.color.g = 20.0;
        marker.color.b = 20.0;
      }
      marker_array.markers.push_back(marker);
    }
    // marker_pub_->publish(marker_array);
    return marker_array;
  }

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PolygonStamped>::SharedPtr fls_window_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_,
    detected_marker_pub_;
  std::vector<geometry_msgs::msg::Point> generated_obstacles, detected_obstacles;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  rclcpp::Publisher<obstacle_pose_interface::msg::ObstaclesInFOVOfSensor>::SharedPtr fls_fov_pub_;

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FlsSimulator>());
  rclcpp::shutdown();
  return 0;
}
