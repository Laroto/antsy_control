#include <algorithm>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "actuator_msgs/msg/actuators.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "kdl/frames.hpp"

#include "antsy_control/tripod_gait.hpp"
#include "antsy_control/leg_odometry.hpp"
#include "antsy_kinematics/kinematics.hpp"
#include "antsy_msgs/msg/gait_phase.hpp"
#include "antsy_msgs/msg/vector3_array.hpp"

using namespace std::placeholders;

class FollowVelocity : public rclcpp::Node
{
public:
  FollowVelocity()
  : Node("follow_velocity")
  {
    loadParameters();
    gait_ = std::make_unique<antsy_control::TripodGait>(gait_params_);
    leg_odometry_ = std::make_unique<antsy_control::LegOdometryEstimator>(leg_odometry_params_);
    resetLegIkSeeds();
    cmd_vel_ = antsy_control::zeroTwist();

    cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
      "cmd_vel", 10, std::bind(&FollowVelocity::cmdVelCallback, this, _1));
    cmd_vel_unstamped_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel_unstamped", 10, std::bind(&FollowVelocity::cmdVelUnstampedCallback, this, _1));
    if (!params_.heading_hold_odom_topic.empty()) {
      heading_hold_odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        params_.heading_hold_odom_topic, 10, std::bind(&FollowVelocity::headingHoldOdomCallback, this, _1));
    }
    body_pose_mode_sub_ = this->create_subscription<std_msgs::msg::Bool>(
      "body_pose_mode", 10, std::bind(&FollowVelocity::bodyPoseModeCallback, this, _1));
    if (!params_.leg_odometry_joint_states_topic.empty()) {
      joint_states_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
        params_.leg_odometry_joint_states_topic, 10,
        std::bind(&FollowVelocity::jointStatesCallback, this, _1));
    }
    go_to_rest_pose_srv_ = this->create_service<std_srvs::srv::Trigger>(
      "go_to_rest_pose",
      std::bind(&FollowVelocity::goToRestPoseCallback, this, _1, _2));
    reset_control_srv_ = this->create_service<std_srvs::srv::Trigger>(
      "/control/reset",
      std::bind(&FollowVelocity::resetControlCallback, this, _1, _2));

    actuators_pub_ = this->create_publisher<actuator_msgs::msg::Actuators>("actuators", 1);
    createJointPositionPublishers();
    cmd_vel_adj_pub_ =
      this->create_publisher<geometry_msgs::msg::TwistStamped>("cmd_vel_adj", 10);
    relative_foot_position_pub_ =
      this->create_publisher<antsy_msgs::msg::Vector3Array>("relative_foot_positions", 10);
    gait_phase_left_pub_ =
      this->create_publisher<antsy_msgs::msg::GaitPhase>("gait_phase_left", 10);
    gait_phase_right_pub_ =
      this->create_publisher<antsy_msgs::msg::GaitPhase>("gait_phase_right", 10);
    leg_odom_pub_ =
      this->create_publisher<nav_msgs::msg::Odometry>(params_.leg_odometry_topic, 10);

    kinematics_ = std::make_shared<antsy_kinematics::Kinematics>(
      std::vector<std::string>{"foot_0", "foot_1", "foot_2", "foot_3", "foot_4", "foot_5"},
      "base_link",
      params_.ik_position_weight,
      params_.ik_orientation_weight);
    robot_description_sub_ = this->create_subscription<std_msgs::msg::String>(
      "robot_description",
      rclcpp::QoS(rclcpp::KeepLast(1)).durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL),
      std::bind(&antsy_kinematics::Kinematics::robotDescriptionCallback, kinematics_.get(), _1));

    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
      "IK: Waiting until URDF received and solvers initialized.");
    while (!kinematics_->isInitialized()) {
      rclcpp::spin_some(this->get_node_base_interface());
      rclcpp::sleep_for(std::chrono::milliseconds(100));
    }

    const auto timer_period =
      std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(params_.control_period));
    timer_ = this->create_wall_timer(
      timer_period, std::bind(&FollowVelocity::timerCallback, this));
    last_update_ = steady_clock_.now();
  }

private:
  struct Parameters
  {
    double control_period;
    double cmd_vel_timeout;
    double max_dt;
    double ik_position_weight;
    double ik_orientation_weight;
    std::vector<double> ik_seed_joint_angles_left;
    std::vector<double> ik_seed_joint_angles_right;
    bool heading_hold_enabled;
    double heading_hold_kp;
    double heading_hold_kd;
    double heading_hold_max_angular_velocity;
    double heading_hold_min_linear_velocity;
    double heading_hold_angular_deadband;
    std::string heading_hold_odom_topic;
    bool leg_odometry_enabled;
    bool leg_odometry_reset_pose_on_control_reset;
    std::string leg_odometry_topic;
    std::string leg_odometry_frame_id;
    std::string leg_odometry_child_frame_id;
    std::string leg_odometry_joint_states_topic;
    bool leg_odometry_use_joint_states;
  };

  void loadParameters()
  {
    params_.control_period =
      this->declare_parameter<double>("control.period", 0.02);
    params_.cmd_vel_timeout =
      this->declare_parameter<double>("cmd_vel_timeout", 0.3);
    params_.max_dt =
      this->declare_parameter<double>("control.max_dt", 0.05);

    gait_params_.max_velocity_ratio =
      this->declare_parameter<double>("gait.max_velocity_ratio", 0.50);
    gait_params_.step_limit_x =
      this->declare_parameter<double>("gait.step_limit_x", 0.045);
    gait_params_.step_limit_y =
      this->declare_parameter<double>("gait.step_limit_y", 0.035);
    gait_params_.foot_z_down =
      this->declare_parameter<double>("gait.foot_z_down", -0.100);
    gait_params_.foot_z_up =
      this->declare_parameter<double>("gait.foot_z_up", -0.070);
    gait_params_.foot_z_sync =
      this->declare_parameter<double>("gait.foot_z_sync", -0.090);
    gait_params_.stance_velocity_x =
      this->declare_parameter<double>("gait.stance_velocity_x", 0.67);
    gait_params_.stance_velocity_y =
      this->declare_parameter<double>("gait.stance_velocity_y", 0.34);
    gait_params_.vertical_velocity =
      this->declare_parameter<double>("gait.vertical_velocity", 0.34 * 2.0 / 3.0);
    gait_params_.swing_xy_velocity =
      this->declare_parameter<double>("gait.swing_xy_velocity", 0.55);
    gait_params_.idle_return_velocity =
      this->declare_parameter<double>("gait.idle_return_velocity", 0.25);
    gait_params_.stop_recenter_tolerance =
      this->declare_parameter<double>("gait.stop_recenter_tolerance", 0.020);
    gait_params_.stop_recenter_velocity =
      this->declare_parameter<double>("gait.stop_recenter_velocity", 0.20);
    gait_params_.ready_stance_enabled =
      this->declare_parameter<bool>("gait.ready_stance_enabled", true);
    gait_params_.ready_stance_linear_x =
      this->declare_parameter<double>("gait.ready_stance_linear_x", 1.0);
    gait_params_.ready_stance_linear_y =
      this->declare_parameter<double>("gait.ready_stance_linear_y", 0.0);
    gait_params_.ready_stance_angular_z =
      this->declare_parameter<double>("gait.ready_stance_angular_z", 0.0);
    gait_params_.max_linear_acceleration =
      this->declare_parameter<double>("command_filter.max_linear_acceleration", 0.8);
    gait_params_.max_angular_acceleration =
      this->declare_parameter<double>("command_filter.max_angular_acceleration", 2.0);
    gait_params_.body_pose_max_x =
      this->declare_parameter<double>("body_pose.max_x", 0.030);
    gait_params_.body_pose_max_y =
      this->declare_parameter<double>("body_pose.max_y", 0.025);
    gait_params_.body_pose_max_z =
      this->declare_parameter<double>("body_pose.max_z", 0.020);
    gait_params_.body_pose_max_roll =
      this->declare_parameter<double>("body_pose.max_roll", 0.18);
    gait_params_.body_pose_max_pitch =
      this->declare_parameter<double>("body_pose.max_pitch", 0.18);
    gait_params_.body_pose_max_yaw =
      this->declare_parameter<double>("body_pose.max_yaw", 0.22);
    gait_params_.body_pose_linear_rate =
      this->declare_parameter<double>("body_pose.linear_rate", 0.10);
    gait_params_.body_pose_angular_rate =
      this->declare_parameter<double>("body_pose.angular_rate", 0.70);
    gait_params_.velocity_epsilon =
      this->declare_parameter<double>("gait.velocity_epsilon", 1e-5);

    params_.ik_position_weight =
      this->declare_parameter<double>("ik.position_weight", 1.0);
    params_.ik_orientation_weight =
      this->declare_parameter<double>("ik.orientation_weight", 0.0);
    params_.ik_seed_joint_angles_left =
      this->declare_parameter<std::vector<double>>(
      "ik.seed_joint_angles_left", std::vector<double>{0.0, 0.6, 1.8});
    params_.ik_seed_joint_angles_right =
      this->declare_parameter<std::vector<double>>(
      "ik.seed_joint_angles_right", std::vector<double>{0.0, -0.6, -1.8});

    params_.heading_hold_enabled =
      this->declare_parameter<bool>("heading_hold.enabled", true);
    params_.heading_hold_kp =
      this->declare_parameter<double>("heading_hold.kp", 1.8);
    params_.heading_hold_kd =
      this->declare_parameter<double>("heading_hold.kd", 0.30);
    params_.heading_hold_max_angular_velocity =
      this->declare_parameter<double>("heading_hold.max_angular_velocity", 0.40);
    params_.heading_hold_min_linear_velocity =
      this->declare_parameter<double>("heading_hold.min_linear_velocity", 0.02);
    params_.heading_hold_angular_deadband =
      this->declare_parameter<double>("heading_hold.angular_deadband", 0.02);
    params_.heading_hold_odom_topic =
      this->declare_parameter<std::string>("heading_hold.odom_topic", "leg_odom");

    params_.leg_odometry_enabled =
      this->declare_parameter<bool>("leg_odometry.enabled", true);
    params_.leg_odometry_reset_pose_on_control_reset =
      this->declare_parameter<bool>("leg_odometry.reset_pose_on_control_reset", true);
    params_.leg_odometry_topic =
      this->declare_parameter<std::string>("leg_odometry.topic", "leg_odom");
    params_.leg_odometry_frame_id =
      this->declare_parameter<std::string>("leg_odometry.frame_id", "odom");
    params_.leg_odometry_child_frame_id =
      this->declare_parameter<std::string>("leg_odometry.child_frame_id", "base_link");
    params_.leg_odometry_joint_states_topic =
      this->declare_parameter<std::string>("leg_odometry.joint_states_topic", "joint_states");
    params_.leg_odometry_use_joint_states =
      this->declare_parameter<bool>("leg_odometry.use_joint_states", true);
    leg_odometry_params_.min_support_legs =
      this->declare_parameter<int>("leg_odometry.min_support_legs", 3);
    leg_odometry_params_.max_fit_residual =
      this->declare_parameter<double>("leg_odometry.max_fit_residual", 0.03);
    leg_odometry_params_.max_linear_delta =
      this->declare_parameter<double>("leg_odometry.max_linear_delta", 0.03);
    leg_odometry_params_.max_angular_delta =
      this->declare_parameter<double>("leg_odometry.max_angular_delta", 0.25);
    leg_odometry_params_.translation_scale =
      this->declare_parameter<double>("leg_odometry.translation_scale", 1.0);
    leg_odometry_params_.propagate_on_invalid_update =
      this->declare_parameter<bool>("leg_odometry.propagate_on_invalid_update", false);
    leg_odometry_params_.max_prediction_time =
      this->declare_parameter<double>("leg_odometry.max_prediction_time", 0.08);

    if (
      params_.control_period <= 0.0 ||
      params_.cmd_vel_timeout < 0.0 ||
      params_.max_dt <= 0.0 ||
      params_.ik_position_weight <= 0.0 ||
      params_.ik_orientation_weight < 0.0 ||
      params_.ik_seed_joint_angles_left.size() != antsy_control::kNumJointsPerLeg ||
      params_.ik_seed_joint_angles_right.size() != antsy_control::kNumJointsPerLeg ||
      params_.heading_hold_kp < 0.0 ||
      params_.heading_hold_kd < 0.0 ||
      params_.heading_hold_max_angular_velocity < 0.0 ||
      params_.heading_hold_min_linear_velocity < 0.0 ||
      params_.heading_hold_angular_deadband < 0.0)
    {
      throw std::runtime_error("Invalid follow_velocity parameters.");
    }
    for (const double joint_angle : params_.ik_seed_joint_angles_left) {
      if (!std::isfinite(joint_angle)) {
        throw std::runtime_error("Invalid IK seed joint angle.");
      }
    }
    for (const double joint_angle : params_.ik_seed_joint_angles_right) {
      if (!std::isfinite(joint_angle)) {
        throw std::runtime_error("Invalid IK seed joint angle.");
      }
    }
  }

  std::vector<std::string> jointNames() const
  {
    std::vector<std::string> names;
    names.reserve(antsy_control::kNumLegs * antsy_control::kNumJointsPerLeg);
    for (int i = 0; i < antsy_control::kNumLegs; i++) {
      names.push_back("leg_" + std::to_string(i) + "_base__leg_" + std::to_string(i) + "a");
      names.push_back("leg_" + std::to_string(i) + "a__leg_" + std::to_string(i) + "b");
      names.push_back("leg_" + std::to_string(i) + "b__leg_" + std::to_string(i) + "c");
    }
    return names;
  }

  void createJointPositionPublishers()
  {
    joint_position_pubs_.clear();
    for (const std::string & joint_name : jointNames()) {
      joint_position_pubs_.push_back(
        this->create_publisher<std_msgs::msg::Float64>(
          "/antsy/joint_position/" + joint_name, 1));
    }
  }

  void resetLegIkSeeds()
  {
    std::vector<antsy_control::Leg> & legs = gait_->mutableLegs();
    for (int i = 0; i < antsy_control::kNumLegs; i++) {
      legs[i].joint_angles = KDL::JntArray(antsy_control::kNumJointsPerLeg);
      const std::vector<double> & seed =
        i < antsy_control::kNumLegs / 2 ?
        params_.ik_seed_joint_angles_left : params_.ik_seed_joint_angles_right;
      for (int j = 0; j < antsy_control::kNumJointsPerLeg; j++) {
        legs[i].joint_angles(j) = seed[j];
      }
    }
  }

  void resetControllerState()
  {
    cmd_vel_ = antsy_control::zeroTwist();
    cmd_vel_stamp_ = this->now();
    heading_hold_active_ = false;
    gait_->reset();
    resetLegIkSeeds();
    leg_odometry_->reset(params_.leg_odometry_reset_pose_on_control_reset);
    last_update_ = steady_clock_.now();
  }

  double getDt()
  {
    const auto now = steady_clock_.now();
    double dt = (now - last_update_).seconds();
    last_update_ = now;

    if (!std::isfinite(dt) || dt <= 0.0) {
      dt = params_.control_period;
    }
    return std::min(dt, params_.max_dt);
  }

  void updateCmdVel(const geometry_msgs::msg::Twist & msg, const rclcpp::Time & stamp)
  {
    cmd_vel_.vel.x(msg.linear.x);
    cmd_vel_.vel.y(msg.linear.y);
    cmd_vel_.vel.z(msg.linear.z);
    cmd_vel_.rot.x(msg.angular.x);
    cmd_vel_.rot.y(msg.angular.y);
    cmd_vel_.rot.z(msg.angular.z);
    cmd_vel_stamp_ = stamp;
  }

  void cmdVelCallback(const geometry_msgs::msg::TwistStamped & msg)
  {
    const rclcpp::Time stamp(msg.header.stamp);
    updateCmdVel(msg.twist, stamp.nanoseconds() == 0 ? this->now() : stamp);
  }

  void cmdVelUnstampedCallback(const geometry_msgs::msg::Twist & msg)
  {
    updateCmdVel(msg, this->now());
  }

  static double yawFromQuaternion(const geometry_msgs::msg::Quaternion & q)
  {
    const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
    const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
    return std::atan2(siny_cosp, cosy_cosp);
  }

  static geometry_msgs::msg::Quaternion quaternionFromYaw(const double yaw)
  {
    geometry_msgs::msg::Quaternion q;
    q.x = 0.0;
    q.y = 0.0;
    q.z = std::sin(yaw * 0.5);
    q.w = std::cos(yaw * 0.5);
    return q;
  }

  static double normalizeAngle(double angle)
  {
    while (angle > M_PI) {
      angle -= 2.0 * M_PI;
    }
    while (angle < -M_PI) {
      angle += 2.0 * M_PI;
    }
    return angle;
  }

  void headingHoldOdomCallback(const nav_msgs::msg::Odometry & msg)
  {
    current_yaw_ = yawFromQuaternion(msg.pose.pose.orientation);
    current_yaw_rate_ = msg.twist.twist.angular.z;
    has_odom_ = true;
  }

  void bodyPoseModeCallback(const std_msgs::msg::Bool & msg)
  {
    if (gait_->bodyPoseModeEnabled() == msg.data) {
      return;
    }

    gait_->setBodyPoseModeEnabled(msg.data);
    heading_hold_active_ = false;
    leg_odometry_->reset(false);
    RCLCPP_INFO(
      this->get_logger(), "Body pose mode %s.", msg.data ? "enabled" : "disabled");
  }

  void jointStatesCallback(const sensor_msgs::msg::JointState & msg)
  {
    std::vector<double> positions(antsy_control::kNumLegs * antsy_control::kNumJointsPerLeg);
    std::vector<bool> seen(positions.size(), false);

    if (msg.name.empty()) {
      if (msg.position.size() < positions.size()) {
        return;
      }
      for (size_t i = 0; i < positions.size(); i++) {
        positions[i] = msg.position[i];
        seen[i] = std::isfinite(positions[i]);
      }
    } else {
      const std::vector<std::string> expected_names = jointNames();
      for (size_t expected = 0; expected < expected_names.size(); expected++) {
        for (size_t actual = 0; actual < msg.name.size() && actual < msg.position.size(); actual++) {
          if (msg.name[actual] != expected_names[expected]) {
            continue;
          }
          positions[expected] = msg.position[actual];
          seen[expected] = std::isfinite(positions[expected]);
          break;
        }
      }
    }

    for (const bool joint_seen : seen) {
      if (!joint_seen) {
        return;
      }
    }
    measured_joint_positions_ = positions;
    has_measured_joint_positions_ = true;
  }

  void goToRestPoseCallback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    gait_->beginRestSequence();
    cmd_vel_ = antsy_control::zeroTwist();
    cmd_vel_stamp_ = this->now();
    heading_hold_active_ = false;
    leg_odometry_->reset(false);
    response->success = true;
    response->message = "Returning to startup resting pose.";
  }

  void resetControlCallback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    resetControllerState();
    response->success = true;
    response->message = "Controller reset to startup state.";
    RCLCPP_WARN(this->get_logger(), "Controller reset to startup state.");
  }

  bool translationCommandActive(const KDL::Twist & command) const
  {
    return std::hypot(command.vel.x(), command.vel.y()) >=
      params_.heading_hold_min_linear_velocity;
  }

  KDL::Twist applyHeadingHold(const KDL::Twist & command)
  {
    KDL::Twist adjusted_command = command;
    const bool can_hold_heading =
      params_.heading_hold_enabled &&
      has_odom_ &&
      translationCommandActive(command) &&
      std::abs(command.rot.z()) <= params_.heading_hold_angular_deadband;

    if (!can_hold_heading) {
      heading_hold_active_ = false;
      return adjusted_command;
    }

    if (!heading_hold_active_) {
      heading_hold_yaw_ = current_yaw_;
      heading_hold_active_ = true;
    }

    const double yaw_error = normalizeAngle(heading_hold_yaw_ - current_yaw_);
    const double correction = std::clamp(
      params_.heading_hold_kp * yaw_error - params_.heading_hold_kd * current_yaw_rate_,
      -params_.heading_hold_max_angular_velocity,
      params_.heading_hold_max_angular_velocity);
    adjusted_command.rot.z(command.rot.z() + correction);
    return adjusted_command;
  }

  KDL::Twist targetCommand(const bool cmd_vel_timed_out)
  {
    if (cmd_vel_timed_out) {
      heading_hold_active_ = false;
      return antsy_control::zeroTwist();
    }
    if (gait_->bodyPoseModeEnabled()) {
      heading_hold_active_ = false;
      return cmd_vel_;
    }
    return applyHeadingHold(cmd_vel_);
  }

  void solveInverseKinematics()
  {
    std::vector<antsy_control::Leg> & legs = gait_->mutableLegs();
    for (int i = 0; i < antsy_control::kNumLegs; i++) {
      const KDL::Vector foot_position =
        legs[i].foot_center_position + legs[i].foot_relative_position;
      KDL::Frame foot_frame(foot_position);
      KDL::JntArray joint_angles_next = legs[i].joint_angles;
      const int ik_result = kinematics_->cartToJnt(
        i, legs[i].joint_angles, foot_frame, joint_angles_next);
      if (ik_result >= 0) {
        kinematics_->foldAndClampJointAnglesToLimits(i, joint_angles_next);
        legs[i].joint_angles = joint_angles_next;
      } else {
        RCLCPP_WARN_THROTTLE(
          this->get_logger(), *this->get_clock(), 1000,
          "IK failed for leg %d, keeping previous joint command.", i);
      }
    }
  }

  void publishActuators(const rclcpp::Time & now)
  {
    const std::vector<antsy_control::Leg> & legs = gait_->legs();
    actuator_msgs::msg::Actuators msg;
    msg.header.stamp = now;
    msg.position.resize(antsy_control::kNumLegs * antsy_control::kNumJointsPerLeg);

    std_msgs::msg::Float64 joint_position_msg;
    for (int i = 0; i < antsy_control::kNumLegs; i++) {
      for (int j = 0; j < antsy_control::kNumJointsPerLeg; j++) {
        const size_t joint_index = i * antsy_control::kNumJointsPerLeg + j;
        msg.position[joint_index] = legs[i].joint_angles(j);
        joint_position_msg.data = msg.position[joint_index];
        joint_position_pubs_[joint_index]->publish(joint_position_msg);
      }
    }
    actuators_pub_->publish(msg);
  }

  void publishAdjustedVelocity(const rclcpp::Time & now)
  {
    const KDL::Twist & smoothed = gait_->smoothedCommand();
    geometry_msgs::msg::TwistStamped msg;
    msg.header.stamp = now;
    msg.twist.linear.x = smoothed.vel.x();
    msg.twist.linear.y = smoothed.vel.y();
    msg.twist.angular.z = smoothed.rot.z();
    cmd_vel_adj_pub_->publish(msg);
  }

  void publishFootPositions(const rclcpp::Time & now)
  {
    const std::vector<antsy_control::Leg> & legs = gait_->legs();
    antsy_msgs::msg::Vector3Array foot_positions_msg;
    foot_positions_msg.header.stamp = now;
    foot_positions_msg.vectors.resize(antsy_control::kNumLegs);
    for (int i = 0; i < antsy_control::kNumLegs; i++) {
      foot_positions_msg.vectors[i].x = legs[i].foot_relative_position.x();
      foot_positions_msg.vectors[i].y = legs[i].foot_relative_position.y();
      foot_positions_msg.vectors[i].z = legs[i].foot_relative_position.z();
    }
    relative_foot_position_pub_->publish(foot_positions_msg);
  }

  void publishGaitPhases(const rclcpp::Time & now)
  {
    antsy_msgs::msg::GaitPhase phase_left_msg;
    phase_left_msg.header.stamp = now;
    phase_left_msg.phase = static_cast<uint8_t>(gait_->leftPhase());
    gait_phase_left_pub_->publish(phase_left_msg);

    antsy_msgs::msg::GaitPhase phase_right_msg;
    phase_right_msg.header.stamp = now;
    phase_right_msg.phase = static_cast<uint8_t>(gait_->rightPhase());
    gait_phase_right_pub_->publish(phase_right_msg);
  }

  void updateAndPublishLegOdometry(const rclcpp::Time & now, const double dt)
  {
    if (!params_.leg_odometry_enabled) {
      return;
    }

    const std::vector<antsy_control::Leg> & legs = gait_->legs();
    std::vector<KDL::Vector> foot_positions(antsy_control::kNumLegs);
    std::vector<bool> stance_legs(antsy_control::kNumLegs);
    const bool use_measured_positions =
      params_.leg_odometry_use_joint_states && has_measured_joint_positions_;

    for (int i = 0; i < antsy_control::kNumLegs; i++) {
      foot_positions[i] = legs[i].foot_center_position + legs[i].foot_relative_position;
      stance_legs[i] = gait_->supportForLeg(i);
    }

    if (use_measured_positions) {
      bool fk_ok = true;
      std::vector<KDL::Vector> measured_foot_positions(antsy_control::kNumLegs);
      for (int i = 0; i < antsy_control::kNumLegs; i++) {
        KDL::JntArray joint_positions(antsy_control::kNumJointsPerLeg);
        for (int j = 0; j < antsy_control::kNumJointsPerLeg; j++) {
          const size_t joint_index = i * antsy_control::kNumJointsPerLeg + j;
          joint_positions(j) = measured_joint_positions_[joint_index];
        }

        KDL::Frame foot_frame;
        if (kinematics_->jntToCart(i, joint_positions, foot_frame) < 0) {
          fk_ok = false;
          break;
        }
        measured_foot_positions[i] = foot_frame.p;
      }
      if (fk_ok) {
        foot_positions = measured_foot_positions;
      } else {
        has_measured_joint_positions_ = false;
      }
    }

    const antsy_control::LegOdometryState state =
      leg_odometry_->update(foot_positions, stance_legs, gait_->bodyPoseModeEnabled(), dt);

    nav_msgs::msg::Odometry msg;
    msg.header.stamp = now;
    msg.header.frame_id = params_.leg_odometry_frame_id;
    msg.child_frame_id = params_.leg_odometry_child_frame_id;
    msg.pose.pose.position.x = state.x;
    msg.pose.pose.position.y = state.y;
    msg.pose.pose.position.z = 0.0;
    msg.pose.pose.orientation = quaternionFromYaw(state.yaw);
    msg.twist.twist.linear.x = state.vx;
    msg.twist.twist.linear.y = state.vy;
    msg.twist.twist.angular.z = state.wz;
    msg.pose.covariance[0] = state.valid_update ? 0.02 : 0.20;
    msg.pose.covariance[7] = state.valid_update ? 0.02 : 0.20;
    msg.pose.covariance[35] = state.valid_update ? 0.05 : 0.50;
    msg.twist.covariance[0] = state.valid_update ? 0.05 : 0.50;
    msg.twist.covariance[7] = state.valid_update ? 0.05 : 0.50;
    msg.twist.covariance[35] = state.valid_update ? 0.10 : 1.00;
    leg_odom_pub_->publish(msg);
  }

  void timerCallback()
  {
    const rclcpp::Time now = this->now();
    const double dt = getDt();
    const double cmd_vel_oldness = (now - cmd_vel_stamp_).seconds();
    const bool cmd_vel_timed_out = cmd_vel_oldness > params_.cmd_vel_timeout;

    if (cmd_vel_timed_out) {
      RCLCPP_INFO_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "Received cmd_vel timed out (%.2f > %.2f s), holding stance.",
        cmd_vel_oldness, params_.cmd_vel_timeout);
    }

    gait_->update(dt, targetCommand(cmd_vel_timed_out), cmd_vel_timed_out);
    updateAndPublishLegOdometry(now, dt);
    solveInverseKinematics();
    publishActuators(now);
    publishAdjustedVelocity(now);
    publishFootPositions(now);
    publishGaitPhases(now);
  }

  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_vel_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_unstamped_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr heading_hold_odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_states_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr body_pose_mode_sub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr go_to_rest_pose_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_control_srv_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr robot_description_sub_;
  rclcpp::Publisher<actuator_msgs::msg::Actuators>::SharedPtr actuators_pub_;
  std::vector<rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr> joint_position_pubs_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_vel_adj_pub_;
  rclcpp::Publisher<antsy_msgs::msg::Vector3Array>::SharedPtr relative_foot_position_pub_;
  rclcpp::Publisher<antsy_msgs::msg::GaitPhase>::SharedPtr gait_phase_left_pub_;
  rclcpp::Publisher<antsy_msgs::msg::GaitPhase>::SharedPtr gait_phase_right_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr leg_odom_pub_;

  rclcpp::TimerBase::SharedPtr timer_;
  std::shared_ptr<antsy_kinematics::Kinematics> kinematics_;
  std::unique_ptr<antsy_control::TripodGait> gait_;
  std::unique_ptr<antsy_control::LegOdometryEstimator> leg_odometry_;

  rclcpp::Clock steady_clock_{RCL_STEADY_TIME};
  rclcpp::Time last_update_{0, 0, RCL_STEADY_TIME};
  KDL::Twist cmd_vel_;
  rclcpp::Time cmd_vel_stamp_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
  bool has_odom_ = false;
  double current_yaw_ = 0.0;
  double current_yaw_rate_ = 0.0;
  bool heading_hold_active_ = false;
  double heading_hold_yaw_ = 0.0;
  std::vector<double> measured_joint_positions_;
  bool has_measured_joint_positions_ = false;
  Parameters params_;
  antsy_control::TripodGaitParameters gait_params_;
  antsy_control::LegOdometryParameters leg_odometry_params_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<FollowVelocity>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
