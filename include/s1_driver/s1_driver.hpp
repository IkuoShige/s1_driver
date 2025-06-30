#ifndef S1_DRIVER_HPP_
#define S1_DRIVER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "s1_driver/rust_bridge.hpp"

#include <memory>
#include <string>
#include <vector>

namespace s1_driver
{

class S1Driver : public rclcpp::Node
{
public:
  explicit S1Driver();
  ~S1Driver();

private:
  // Callback functions
  void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
  void timerCallback();
  
  // CAN communication functions
  bool initializeCAN();
  void shutdownCAN();
  bool sendMovementCommand(double vx, double vy, double vz);
  bool readSensorData();
  
  // Data processing functions
  void updateOdometry();
  void publishTF();
  void publishSensorData();
  
  // ROS2 publishers and subscribers
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
  
  // TF broadcaster
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  
  // Timer for periodic updates
  rclcpp::TimerBase::SharedPtr timer_;
  
  // Robot state variables
  struct RobotState {
    double x, y, theta;           // Position and orientation
    double vx, vy, vtheta;        // Velocities
    double wheel_positions[4];    // Wheel positions [rad]
    double wheel_velocities[4];   // Wheel velocities [rad/s]
    double imu_accel[3];         // Linear acceleration [m/s²]
    double imu_gyro[3];          // Angular velocity [rad/s]
    rclcpp::Time timestamp;
  } robot_state_;
  
  // Parameters
  std::string can_interface_;
  std::string base_frame_;
  std::string odom_frame_;
  double control_frequency_;
  double cmd_vel_timeout_;
  bool publish_tf_;
  
  // Safety and control variables
  rclcpp::Time last_cmd_vel_time_;
  bool emergency_stop_;
  bool can_initialized_;
  
  // Rust bridge for CAN communication
  std::unique_ptr<RustBridge> rust_bridge_;
  
  // Constants
  static constexpr double WHEEL_RADIUS = 0.05;        // meters
  static constexpr double WHEEL_BASE_WIDTH = 0.2;     // meters  
  static constexpr double WHEEL_BASE_LENGTH = 0.2;    // meters
  static constexpr double MAX_LINEAR_VELOCITY = 2.0;  // m/s
  static constexpr double MAX_ANGULAR_VELOCITY = 3.0; // rad/s
};

} // namespace s1_driver

#endif // S1_DRIVER_HPP_
