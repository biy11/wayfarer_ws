#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/twist.hpp>

#include <algorithm>
#include <cmath>
#include <numeric>
#include <vector>
#include <string>

class ObstAvoidance : public rclcpp::Node {
public:
  ObstAvoidance()
  : Node("ObstAvoidance"),
    left_average_(std::numeric_limits<float>::quiet_NaN()),
    right_average_(std::numeric_limits<float>::quiet_NaN()),
    front_average_(std::numeric_limits<float>::quiet_NaN()) {

    // ---- Parameters (tunable at runtime) ----
    this->declare_parameter<double>("safe_distance", 0.2);     // m
    this->declare_parameter<double>("linear_speed", 1.0);      // m/s
    this->declare_parameter<double>("turn_speed", 0.8);        // rad/s
    this->declare_parameter<double>("balance_tolerance", 0.2); // m (how close L/R must be to count as "similar")

    safe_distance_      = this->get_parameter("safe_distance").as_double();
    linear_speed_       = this->get_parameter("linear_speed").as_double();
    turn_speed_         = this->get_parameter("turn_speed").as_double();
    balance_tolerance_  = this->get_parameter("balance_tolerance").as_double();

    // ---- Publishers ----
    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    // ---- Subscriptions ----
    lidar_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan", 10, std::bind(&ObstAvoidance::lidar_callback, this, std::placeholders::_1));
  }

private:
  // Convert degrees to radians
  static inline double deg2rad(double deg) { return deg * M_PI / 180.0; }

  void lidar_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    // Define sectors relative to forward = 0°
    // front: [-15°, +15°], left: [+75°, +105°], right: [-105°, -75°]
    front_average_ = get_average_distance(msg, -15.0, +15.0);
    left_average_  = get_average_distance(msg, +75.0, +105.0);
    right_average_ = get_average_distance(msg, -105.0, -75.0);

    RCLCPP_INFO(this->get_logger(), "----------------------------------------");
    RCLCPP_INFO(this->get_logger(),
                "avg(m): front=%.2f  left=%.2f  right=%.2f",
                front_average_, left_average_, right_average_);
    RCLCPP_INFO(this->get_logger(), "----------------------------------------");
    RCLCPP_INFO(this->get_logger(), "");

    navigate();  // actively command the robot
    //RCLCPP_INFO(this->get_logger(), "New file");
  }

  float get_average_distance(const sensor_msgs::msg::LaserScan::SharedPtr &msg,
                             double angle_start_deg, double angle_end_deg) {
    const auto &ranges = msg->ranges;

    const double start_rad = deg2rad(angle_start_deg);
    const double end_rad   = deg2rad(angle_end_deg);

    // Compute indices (clamped to bounds)
    const int idx_start = std::max(
        0, static_cast<int>(std::floor((start_rad - msg->angle_min) / msg->angle_increment)));
    const int idx_end = std::min(
        static_cast<int>(ranges.size()),
        static_cast<int>(std::floor((end_rad - msg->angle_min) / msg->angle_increment)));

    if (idx_end <= idx_start || ranges.empty()) {
      return std::numeric_limits<float>::quiet_NaN();
    }

    // Collect valid (finite, within range limits) samples
    std::vector<float> valid;
    valid.reserve(static_cast<size_t>(idx_end - idx_start));
    for (int i = idx_start; i < idx_end; ++i) {
      const float v = ranges[i];
      if (std::isfinite(v) && v >= msg->range_min && v <= msg->range_max) {
        valid.push_back(v);
      }
    }

    if (valid.empty()) {
      // If nothing valid, treat as "no obstacle" = max range
      return msg->range_max;
    }

    const double sum = std::accumulate(valid.begin(), valid.end(), 0.0);
    return static_cast<float>(sum / static_cast<double>(valid.size()));
  }

  void rover_move(double linear, double angular) {
    geometry_msgs::msg::Twist msg;
    msg.linear.x  = linear;
    msg.angular.z = angular;
    cmd_vel_pub_->publish(msg);
  }

  // void navigate() {
  //   // Basic rule-based navigation:
  //   //
  //   // 1) If obstacle ahead (closer than safe_distance_), decide turn direction:
  //   //    - If left and right are similar (|L-R| < balance_tolerance_), default left turn
  //   //    - Else turn toward the side with more space (larger average)
  //   // 2) Otherwise, go forward

  //   const bool obstacle_ahead = (front_average_ <= static_cast<float>(safe_distance_));

  //   if (obstacle_ahead) {
  //     const double diff_lr = std::fabs(static_cast<double>(right_average_ - left_average_));

  //     if (diff_lr < balance_tolerance_) {
  //       RCLCPP_INFO(this->get_logger(), "Obstacle ahead → turning left (default)");
  //       rover_move(0.0, +turn_speed_);
  //     } else if (right_average_ > left_average_) {
  //       RCLCPP_INFO(this->get_logger(), "Obstacle ahead → turning right (more space on right)");
  //       rover_move(0.0, -turn_speed_);
  //     } else {
  //       RCLCPP_INFO(this->get_logger(), "Obstacle ahead → turning left (more space on left)");
  //       rover_move(0.0, +turn_speed_);
  //     }
  //   } else {
  //     RCLCPP_INFO(this->get_logger(), "Path clear → moving forward");
  //     rover_move(linear_speed_, 0.0);
  //   }
  // }

  void navigate(){
     
    const bool obst_ahead = (front_average_ <= static_cast<float>(safe_distance_));
    const bool obst_left = (left_average_ <= static_cast<float>(safe_distance_ - 0.2));
    const bool obst_right = (right_average_ <= static_cast<float>(safe_distance_ -0.2));

    return;
    if(obst_left < 0.4){
      RCLCPP_INFO(this->get_logger(), "Path clear -> moving forward [distance : %.2f]", left_average_);
      rover_move(linear_speed_, 0.0);
    }else{
      rover_move(0.0,0.0);
      RCLCPP_INFO(this->get_logger(), "Obstancle detceted -> stopping robot [ditsance : %.2f]", left_average_);
    }
  }

  // Latest sector averages
  float left_average_;
  float right_average_;
  float front_average_;

  // Tunables
  double safe_distance_;
  double linear_speed_;
  double turn_speed_;
  double balance_tolerance_;

  // ROS interfaces
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr       cmd_vel_pub_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ObstAvoidance>());
  rclcpp::shutdown();
  return 0;
}
