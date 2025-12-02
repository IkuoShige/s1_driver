#include "s1_driver/s1_driver.hpp"
#include <chrono>
#include <cmath>
#include <algorithm>

using namespace std::chrono_literals;

namespace s1_driver
{

S1Driver::S1Driver() : Node("s1_driver")
{
  // Declare parameters
  this->declare_parameter<std::string>("can_interface", "can2");
  this->declare_parameter<std::string>("base_frame", "base_link");
  this->declare_parameter<std::string>("odom_frame", "odom");
  this->declare_parameter<double>("control_frequency", 50.0);
  this->declare_parameter<double>("cmd_vel_timeout", 1.0);
  this->declare_parameter<bool>("publish_tf", true);
  
  // Get parameters
  can_interface_ = this->get_parameter("can_interface").as_string();
  base_frame_ = this->get_parameter("base_frame").as_string();
  odom_frame_ = this->get_parameter("odom_frame").as_string();
  control_frequency_ = this->get_parameter("control_frequency").as_double();
  cmd_vel_timeout_ = this->get_parameter("cmd_vel_timeout").as_double();
  publish_tf_ = this->get_parameter("publish_tf").as_bool();
  
  RCLCPP_INFO(this->get_logger(), "S1 Driver starting with CAN interface: %s", can_interface_.c_str());
  
  // Initialize state
  robot_state_ = {};
  robot_state_.timestamp = this->now();
  emergency_stop_ = false;
  can_initialized_ = false;
  last_cmd_vel_time_ = this->now();
  
  // Initialize Rust bridge
  rust_bridge_ = std::make_unique<RustBridge>(can_interface_);
  
  // Create publishers
  odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);
  imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("imu/data", 10);
  joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
  
  // Create subscriber
  cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
    "cmd_vel", 10,
    std::bind(&S1Driver::cmdVelCallback, this, std::placeholders::_1));
  
  // Create TF broadcaster
  if (publish_tf_) {
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
  }
  
  // Initialize CAN communication
  if (!initializeCAN()) {
    RCLCPP_ERROR(this->get_logger(), "Failed to initialize CAN communication");
    return;
  }
  
  // Create timer for periodic updates
  auto timer_period = std::chrono::duration<double>(1.0 / control_frequency_);
  timer_ = this->create_wall_timer(
    timer_period,
    std::bind(&S1Driver::timerCallback, this));
  
  RCLCPP_INFO(this->get_logger(), "S1 Driver initialized successfully");
}

S1Driver::~S1Driver()
{
  shutdownCAN();
}

void S1Driver::cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  if (emergency_stop_ || !can_initialized_) {
    return;
  }
  
  // Update last command time
  last_cmd_vel_time_ = this->now();
  
  // Extract velocities
  double vx = std::clamp(msg->linear.x, -MAX_LINEAR_VELOCITY, MAX_LINEAR_VELOCITY);
  double vy = std::clamp(msg->linear.y, -MAX_LINEAR_VELOCITY, MAX_LINEAR_VELOCITY);
  double vz = std::clamp(msg->angular.z, -MAX_ANGULAR_VELOCITY, MAX_ANGULAR_VELOCITY);
  
  // Send movement command
  if (!sendMovementCommand(vx, vy, vz)) {
    RCLCPP_WARN(this->get_logger(), "Failed to send movement command");
  }
}

void S1Driver::timerCallback()
{
  if (!can_initialized_) {
    return;
  }
  
  // Check for command timeout
  auto current_time = this->now();
  auto time_since_last_cmd = (current_time - last_cmd_vel_time_).seconds();
  
  if (time_since_last_cmd > cmd_vel_timeout_) {
    // Send stop command
    sendMovementCommand(0.0, 0.0, 0.0);
  }
  
  // Read sensor data from robot
  if (readSensorData()) {
    updateOdometry();
    publishSensorData();
    
    if (publish_tf_) {
      publishTF();
    }
  }
}

bool S1Driver::initializeCAN()
{
  RCLCPP_INFO(this->get_logger(), "Initializing CAN communication...");
  
  if (!rust_bridge_) {
    RCLCPP_ERROR(this->get_logger(), "Rust bridge not initialized");
    return false;
  }
  
  if (!rust_bridge_->initialize()) {
    RCLCPP_ERROR(this->get_logger(), "Failed to initialize Rust bridge: %s", 
                 rust_bridge_->getLastError().c_str());
    return false;
  }
  
  can_initialized_ = true;
  RCLCPP_INFO(this->get_logger(), "CAN communication initialized successfully");
  return true;
}

void S1Driver::shutdownCAN()
{
  if (can_initialized_) {
    RCLCPP_INFO(this->get_logger(), "Shutting down CAN communication");
    
    // Stop the robot before shutdown
    if (rust_bridge_) {
      rust_bridge_->stop();
    }
    
    can_initialized_ = false;
  }
}

bool S1Driver::sendMovementCommand(double vx, double vy, double vz)
{
  if (!rust_bridge_ || !can_initialized_) {
    return false;
  }
  
  RCLCPP_DEBUG(this->get_logger(), 
    "Sending movement command: vx=%.2f, vy=%.2f, vz=%.2f", vx, vy, vz);
  
  // Normalize velocities to -1.0 to 1.0 range expected by robomaster-rust
  double norm_vx = vx / MAX_LINEAR_VELOCITY;
  double norm_vy = vy / MAX_LINEAR_VELOCITY;
  double norm_vz = vz / MAX_ANGULAR_VELOCITY;
  
  if (!rust_bridge_->move(norm_vx, norm_vy, norm_vz)) {
    RCLCPP_WARN(this->get_logger(), "Failed to send movement command: %s",
                rust_bridge_->getLastError().c_str());
    return false;
  }
  
  return true;
}

bool S1Driver::readSensorData()
{
  if (!rust_bridge_ || !can_initialized_) {
    return false;
  }
  
  SensorData sensor_data;
  if (!rust_bridge_->readSensors(sensor_data)) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                         "Failed to read sensor data: %s",
                         rust_bridge_->getLastError().c_str());
    return false;
  }
  
  // Update robot state with sensor data
  robot_state_.timestamp = this->now();
  robot_state_.imu_accel[0] = sensor_data.accel_x;
  robot_state_.imu_accel[1] = sensor_data.accel_y;
  robot_state_.imu_accel[2] = sensor_data.accel_z;
  robot_state_.imu_gyro[0] = sensor_data.gyro_x;
  robot_state_.imu_gyro[1] = sensor_data.gyro_y;
  robot_state_.imu_gyro[2] = sensor_data.gyro_z;
  
  // Convert wheel speeds from rad/s to position integration (simple integration)
  static auto last_time = robot_state_.timestamp;
  double dt = (robot_state_.timestamp - last_time).seconds();
  if (dt > 0) {
    for (int i = 0; i < 4; ++i) {
      robot_state_.wheel_velocities[i] = sensor_data.wheel_speeds[i];
      robot_state_.wheel_positions[i] += sensor_data.wheel_speeds[i] * dt;
    }
  }
  last_time = robot_state_.timestamp;
  
  // Simulate wheel encoder data
  for (int i = 0; i < 4; ++i) {
    robot_state_.wheel_velocities[i] = 0.0;  // Will be updated from real sensors
  }
  
  // Simulate IMU data
  robot_state_.imu_accel[0] = 0.0;
  robot_state_.imu_accel[1] = 0.0;
  robot_state_.imu_accel[2] = 9.81;  // Gravity
  
  robot_state_.imu_gyro[0] = 0.0;
  robot_state_.imu_gyro[1] = 0.0;
  robot_state_.imu_gyro[2] = 0.0;
  
  return true;
}

void S1Driver::updateOdometry()
{
  // TODO: Implement odometry calculation from wheel encoders
  // This is a placeholder that maintains current position
  
  auto current_time = robot_state_.timestamp;
  static auto last_time = current_time;
  
  double dt = (current_time - last_time).seconds();
  if (dt <= 0.0) return;
  
  // Placeholder odometry integration
  robot_state_.x += robot_state_.vx * dt;
  robot_state_.y += robot_state_.vy * dt;
  robot_state_.theta += robot_state_.vtheta * dt;
  
  last_time = current_time;
}

void S1Driver::publishSensorData()
{
  auto current_time = robot_state_.timestamp;
  
  // Publish odometry
  auto odom_msg = nav_msgs::msg::Odometry();
  odom_msg.header.stamp = current_time;
  odom_msg.header.frame_id = odom_frame_;
  odom_msg.child_frame_id = base_frame_;
  
  // Position
  odom_msg.pose.pose.position.x = robot_state_.x;
  odom_msg.pose.pose.position.y = robot_state_.y;
  odom_msg.pose.pose.position.z = 0.0;
  
  // Orientation
  tf2::Quaternion q;
  q.setRPY(0, 0, robot_state_.theta);
  odom_msg.pose.pose.orientation = tf2::toMsg(q);
  
  // Velocity
  odom_msg.twist.twist.linear.x = robot_state_.vx;
  odom_msg.twist.twist.linear.y = robot_state_.vy;
  odom_msg.twist.twist.angular.z = robot_state_.vtheta;
  
  odom_pub_->publish(odom_msg);
  
  // Publish IMU data
  auto imu_msg = sensor_msgs::msg::Imu();
  imu_msg.header.stamp = current_time;
  imu_msg.header.frame_id = base_frame_;
  
  imu_msg.linear_acceleration.x = robot_state_.imu_accel[0];
  imu_msg.linear_acceleration.y = robot_state_.imu_accel[1];
  imu_msg.linear_acceleration.z = robot_state_.imu_accel[2];
  
  imu_msg.angular_velocity.x = robot_state_.imu_gyro[0];
  imu_msg.angular_velocity.y = robot_state_.imu_gyro[1];
  imu_msg.angular_velocity.z = robot_state_.imu_gyro[2];
  
  imu_pub_->publish(imu_msg);
  
  // Publish joint states
  auto joint_msg = sensor_msgs::msg::JointState();
  joint_msg.header.stamp = current_time;
  joint_msg.name = {"front_left_wheel_joint", "front_right_wheel_joint", 
                    "rear_left_wheel_joint", "rear_right_wheel_joint"};
  
  joint_msg.position.resize(4);
  joint_msg.velocity.resize(4);
  
  for (int i = 0; i < 4; ++i) {
    joint_msg.position[i] = robot_state_.wheel_positions[i];
    joint_msg.velocity[i] = robot_state_.wheel_velocities[i];
  }
  
  joint_state_pub_->publish(joint_msg);
}

void S1Driver::publishTF()
{
  if (!tf_broadcaster_) return;
  
  geometry_msgs::msg::TransformStamped transform;
  transform.header.stamp = robot_state_.timestamp;
  transform.header.frame_id = odom_frame_;
  transform.child_frame_id = base_frame_;
  
  transform.transform.translation.x = robot_state_.x;
  transform.transform.translation.y = robot_state_.y;
  transform.transform.translation.z = 0.0;
  
  tf2::Quaternion q;
  q.setRPY(0, 0, robot_state_.theta);
  transform.transform.rotation = tf2::toMsg(q);
  
  tf_broadcaster_->sendTransform(transform);
}

} // namespace s1_driver
