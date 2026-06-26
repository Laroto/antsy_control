#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "actuator_msgs/msg/actuators.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "kdl/frames.hpp"
#include "kdl/jntarray.hpp"

#include "antsy_kinematics/kinematics.hpp"
#include "antsy_msgs/msg/vector3_array.hpp"
#include "antsy_msgs/msg/gait_phase.hpp"


using namespace std::placeholders;

enum class GaitPhase {
  DOWN      = 0,
  RISING    = 1,
  UP        = 2,
  FALLING   = 3,
};

enum class StopState {
  WALKING         = 0,
  LANDING         = 1,
  REPLANT_LEFT    = 2,
  REPLANT_RIGHT   = 3,
  HOLDING         = 4,
};

enum class RestState {
  LANDING         = 0,
  REPLANT_LEFT    = 1,
  REPLANT_RIGHT   = 2,
  HOLDING         = 3,
};

struct Leg {
  // neutral position, expressed in robot base_link
  KDL::Vector foot_center_position;
  // last foot position, expressed relative to neutral position
  KDL::Vector foot_relative_position;
  KDL::Vector swing_start_position;
  bool in_continuous_swing = false;
  // last IK solution (to initialize the IK solver next time)
  KDL::JntArray joint_angles;
};

class FollowVelocity : public rclcpp::Node
{
public:
  FollowVelocity()
  : Node("follow_velocity")
  {
    loadParameters();
    cmd_vel_ = zeroTwist();
    cmd_vel_smoothed_ = zeroTwist();
    body_pose_current_ = zeroTwist();

    // Listen to velocity setpoint
    cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
      "cmd_vel", 10, std::bind(&FollowVelocity::cmdVelCallback, this, _1));
    cmd_vel_unstamped_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel_unstamped", 10, std::bind(&FollowVelocity::cmdVelUnstampedCallback, this, _1));
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "odom", 10, std::bind(&FollowVelocity::odomCallback, this, _1));
    body_pose_mode_sub_ = this->create_subscription<std_msgs::msg::Bool>(
      "body_pose_mode", 10, std::bind(&FollowVelocity::bodyPoseModeCallback, this, _1));
    go_to_rest_pose_srv_ = this->create_service<std_srvs::srv::Trigger>(
      "go_to_rest_pose",
      std::bind(&FollowVelocity::goToRestPoseCallback, this, _1, _2));
    reset_control_srv_ = this->create_service<std_srvs::srv::Trigger>(
      "/control/reset",
      std::bind(&FollowVelocity::resetControlCallback, this, _1, _2));
    // Publisher for joint angle references
    actuators_pub_ = this->create_publisher<actuator_msgs::msg::Actuators>(
      "actuators", 1);
    createJointPositionPublishers();
    // Adjusted velocity
    cmd_vel_adj_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(
      "cmd_vel_adj", 10);
    // Cartesian relative foot positions
    relative_foot_position_pub_ =
      this->create_publisher<antsy_msgs::msg::Vector3Array>(
        "relative_foot_positions", 10);
    // Gait phase
    gait_phase_left_pub_ =
      this->create_publisher<antsy_msgs::msg::GaitPhase>(
        "gait_phase_left", 10);
    gait_phase_right_pub_ =
      this->create_publisher<antsy_msgs::msg::GaitPhase>(
        "gait_phase_right", 10);

    // Initialize legs
    legs_.resize(nb_legs_);
    for (int i = 0; i < nb_legs_; i++) {
      legs_[i].foot_center_position.x(
        (i == 0 || i == 5 ? 0.18 : 0.00) + (i == 2 || i == 3 ? -0.18 : 0.00));
      legs_[i].foot_center_position.y(
        (i == 1 || i == 4 ? 0.27 : 0.20) * (i < 3 ? 1 : -1));
      legs_[i].foot_center_position.z(0);
      legs_[i].foot_relative_position.x(0);
      legs_[i].foot_relative_position.y(0);
      legs_[i].foot_relative_position.z(params_.foot_z_down);
      legs_[i].swing_start_position = legs_[i].foot_relative_position;
      legs_[i].joint_angles = KDL::JntArray(nb_joints_per_leg_);
      const std::vector<double> & seed =
        i < nb_legs_ / 2 ? params_.ik_seed_joint_angles_left : params_.ik_seed_joint_angles_right;
      for (int j = 0; j < nb_joints_per_leg_; j++) {
        legs_[i].joint_angles(j) = seed[j];
      }
    }
    rest_pose_targets_.resize(nb_legs_);
    for (int i = 0; i < nb_legs_; i++) {
      rest_pose_targets_[i] = legs_[i].foot_relative_position;
    }
    left_phase_ = GaitPhase::DOWN;
    right_phase_ = GaitPhase::DOWN;

    // Start listening to URDF, create KDL tree,
    // extract chains and construct a solver for each
    kinematics_ = std::make_shared<antsy_kinematics::Kinematics>(
      std::vector<std::string>{"foot_0", "foot_1", "foot_2", "foot_3", "foot_4", "foot_5"},
      "base_link",
      params_.ik_position_weight,
      params_.ik_orientation_weight);
    // Subscribe to robot_description here and forward messages to the kinematics helper
    robot_description_sub_ = this->create_subscription<std_msgs::msg::String>(
      "robot_description",
      rclcpp::QoS(rclcpp::KeepLast(1)).durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL),
      std::bind(&antsy_kinematics::Kinematics::robotDescriptionCallback, kinematics_.get(), _1));

    // Wait until kinematics solvers are initialized; caller drives callbacks
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
      "IK: Waiting until URDF received and solvers initialized.");
    while (!kinematics_->isInitialized()) {
      rclcpp::spin_some(this->get_node_base_interface());
      rclcpp::sleep_for(std::chrono::milliseconds(100));
    }

    // Main timer callback, which moves the legs
    const auto timer_period =
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(params_.control_period));
    timer_ = this->create_wall_timer(
      timer_period, std::bind(&FollowVelocity::timerCallback, this));
    last_update_ = steady_clock_.now();

  }
private:
  std::vector<std::string> jointNames() const
  {
    std::vector<std::string> names;
    names.reserve(nb_legs_ * nb_joints_per_leg_);
    for (int i = 0; i < nb_legs_; i++) {
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

  struct Parameters {
    double control_period;
    double cmd_vel_timeout;
    double max_velocity_ratio;
    double step_limit_x;
    double step_limit_y;
    double foot_z_down;
    double foot_z_up;
    double foot_z_sync;
    double stance_velocity_x;
    double stance_velocity_y;
    double vertical_velocity;
    double swing_xy_velocity;
    double idle_return_velocity;
    double stop_recenter_tolerance;
    double stop_recenter_velocity;
    bool ready_stance_enabled;
    double ready_stance_linear_x;
    double ready_stance_linear_y;
    double ready_stance_angular_z;
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
    double max_linear_acceleration;
    double max_angular_acceleration;
    double body_pose_max_x;
    double body_pose_max_y;
    double body_pose_max_z;
    double body_pose_max_roll;
    double body_pose_max_pitch;
    double body_pose_max_yaw;
    double body_pose_linear_rate;
    double body_pose_angular_rate;
    double velocity_epsilon;
    double max_dt;
  };

  void loadParameters()
  {
    params_.control_period =
      this->declare_parameter<double>("control.period", 0.02);
    params_.cmd_vel_timeout =
      this->declare_parameter<double>("cmd_vel_timeout", 0.3);
    params_.max_velocity_ratio =
      this->declare_parameter<double>("gait.max_velocity_ratio", 0.50);
    params_.step_limit_x =
      this->declare_parameter<double>("gait.step_limit_x", 0.045);
    params_.step_limit_y =
      this->declare_parameter<double>("gait.step_limit_y", 0.035);
    params_.foot_z_down =
      this->declare_parameter<double>("gait.foot_z_down", -0.100);
    params_.foot_z_up =
      this->declare_parameter<double>("gait.foot_z_up", -0.070);
    params_.foot_z_sync =
      this->declare_parameter<double>("gait.foot_z_sync", -0.090);
    params_.stance_velocity_x =
      this->declare_parameter<double>("gait.stance_velocity_x", 0.67);
    params_.stance_velocity_y =
      this->declare_parameter<double>("gait.stance_velocity_y", 0.34);
    params_.vertical_velocity =
      this->declare_parameter<double>("gait.vertical_velocity", 0.34 * 2.0 / 3.0);
    params_.swing_xy_velocity =
      this->declare_parameter<double>("gait.swing_xy_velocity", 0.55);
    params_.idle_return_velocity =
      this->declare_parameter<double>("gait.idle_return_velocity", 0.25);
    params_.stop_recenter_tolerance =
      this->declare_parameter<double>("gait.stop_recenter_tolerance", 0.020);
    params_.stop_recenter_velocity =
      this->declare_parameter<double>("gait.stop_recenter_velocity", 0.20);
    params_.ready_stance_enabled =
      this->declare_parameter<bool>("gait.ready_stance_enabled", true);
    params_.ready_stance_linear_x =
      this->declare_parameter<double>("gait.ready_stance_linear_x", 1.0);
    params_.ready_stance_linear_y =
      this->declare_parameter<double>("gait.ready_stance_linear_y", 0.0);
    params_.ready_stance_angular_z =
      this->declare_parameter<double>("gait.ready_stance_angular_z", 0.0);
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
    params_.max_linear_acceleration =
      this->declare_parameter<double>("command_filter.max_linear_acceleration", 0.8);
    params_.max_angular_acceleration =
      this->declare_parameter<double>("command_filter.max_angular_acceleration", 2.0);
    params_.body_pose_max_x =
      this->declare_parameter<double>("body_pose.max_x", 0.030);
    params_.body_pose_max_y =
      this->declare_parameter<double>("body_pose.max_y", 0.025);
    params_.body_pose_max_z =
      this->declare_parameter<double>("body_pose.max_z", 0.020);
    params_.body_pose_max_roll =
      this->declare_parameter<double>("body_pose.max_roll", 0.18);
    params_.body_pose_max_pitch =
      this->declare_parameter<double>("body_pose.max_pitch", 0.18);
    params_.body_pose_max_yaw =
      this->declare_parameter<double>("body_pose.max_yaw", 0.22);
    params_.body_pose_linear_rate =
      this->declare_parameter<double>("body_pose.linear_rate", 0.10);
    params_.body_pose_angular_rate =
      this->declare_parameter<double>("body_pose.angular_rate", 0.70);
    params_.velocity_epsilon =
      this->declare_parameter<double>("gait.velocity_epsilon", 1e-5);
    params_.max_dt =
      this->declare_parameter<double>("control.max_dt", 0.05);

    if (params_.control_period <= 0.0 ||
      params_.step_limit_x <= 0.0 ||
      params_.step_limit_y <= 0.0 ||
      params_.stance_velocity_x <= 0.0 ||
      params_.stance_velocity_y <= 0.0 ||
      params_.vertical_velocity <= 0.0 ||
      params_.swing_xy_velocity <= 0.0 ||
      params_.idle_return_velocity <= 0.0 ||
      params_.stop_recenter_tolerance < 0.0 ||
      params_.stop_recenter_velocity <= 0.0 ||
      !std::isfinite(params_.ready_stance_linear_x) ||
      !std::isfinite(params_.ready_stance_linear_y) ||
      !std::isfinite(params_.ready_stance_angular_z) ||
      params_.ik_position_weight <= 0.0 ||
      params_.ik_orientation_weight < 0.0 ||
      params_.ik_seed_joint_angles_left.size() != nb_joints_per_leg_ ||
      params_.ik_seed_joint_angles_right.size() != nb_joints_per_leg_ ||
      params_.heading_hold_kp < 0.0 ||
      params_.heading_hold_kd < 0.0 ||
      params_.heading_hold_max_angular_velocity < 0.0 ||
      params_.heading_hold_min_linear_velocity < 0.0 ||
      params_.heading_hold_angular_deadband < 0.0 ||
      params_.max_linear_acceleration <= 0.0 ||
      params_.max_angular_acceleration <= 0.0 ||
      params_.body_pose_max_x < 0.0 ||
      params_.body_pose_max_y < 0.0 ||
      params_.body_pose_max_z < 0.0 ||
      params_.body_pose_max_roll < 0.0 ||
      params_.body_pose_max_pitch < 0.0 ||
      params_.body_pose_max_yaw < 0.0 ||
      params_.body_pose_linear_rate <= 0.0 ||
      params_.body_pose_angular_rate <= 0.0 ||
      params_.foot_z_up <= params_.foot_z_down)
    {
      throw std::runtime_error("Invalid follow_velocity gait parameters.");
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

  inline GaitPhase getLegPhase(const int i)
  {
    // TODO generalize this to other numbers than six legs
    return i % 2 ? right_phase_ : left_phase_;
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

  void odomCallback(const nav_msgs::msg::Odometry & msg)
  {
    current_yaw_ = yawFromQuaternion(msg.pose.pose.orientation);
    current_yaw_rate_ = msg.twist.twist.angular.z;
    has_odom_ = true;
  }

  void bodyPoseModeCallback(const std_msgs::msg::Bool & msg)
  {
    if (body_pose_mode_enabled_ == msg.data) {
      return;
    }

    body_pose_mode_enabled_ = msg.data;
    rest_sequence_active_ = false;
    cmd_vel_smoothed_ = zeroTwist();
    body_pose_current_ = zeroTwist();
    heading_hold_active_ = false;
    clearContinuousSwingState();
    left_phase_ = GaitPhase::DOWN;
    right_phase_ = GaitPhase::DOWN;
    stop_state_ = StopState::HOLDING;
    RCLCPP_INFO(
      this->get_logger(), "Body pose mode %s.", body_pose_mode_enabled_ ? "enabled" : "disabled");
  }

  KDL::Twist zeroTwist() const
  {
    return KDL::Twist(KDL::Vector(0.0, 0.0, 0.0), KDL::Vector(0.0, 0.0, 0.0));
  }

  void resetLegIkSeeds()
  {
    for (int i = 0; i < nb_legs_; i++) {
      legs_[i].joint_angles = KDL::JntArray(nb_joints_per_leg_);
      const std::vector<double> & seed =
        i < nb_legs_ / 2 ? params_.ik_seed_joint_angles_left : params_.ik_seed_joint_angles_right;
      for (int j = 0; j < nb_joints_per_leg_; j++) {
        legs_[i].joint_angles(j) = seed[j];
      }
    }
  }

  void resetControllerState()
  {
    cmd_vel_ = zeroTwist();
    cmd_vel_smoothed_ = zeroTwist();
    body_pose_current_ = zeroTwist();
    cmd_vel_stamp_ = this->now();
    heading_hold_active_ = false;
    body_pose_mode_enabled_ = false;
    stop_state_ = StopState::HOLDING;
    rest_sequence_active_ = false;
    rest_state_ = RestState::HOLDING;
    gait_cycle_phase_ = 0.0;
    start_with_right_tripod_ = true;
    left_phase_ = GaitPhase::DOWN;
    right_phase_ = GaitPhase::DOWN;
    last_update_ = steady_clock_.now();

    for (int i = 0; i < nb_legs_; i++) {
      legs_[i].foot_relative_position = rest_pose_targets_[i];
      legs_[i].swing_start_position = legs_[i].foot_relative_position;
      legs_[i].in_continuous_swing = false;
    }
    resetLegIkSeeds();
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

  double moveTowards(const double current, const double target, const double max_delta) const
  {
    const double delta = target - current;
    if (std::abs(delta) <= max_delta) {
      return target;
    }
    return current + std::copysign(max_delta, delta);
  }

  void clampFootXY(KDL::Vector & p) const
  {
    p.x(std::clamp(p.x(), -params_.step_limit_x, params_.step_limit_x));
    p.y(std::clamp(p.y(), -params_.step_limit_y, params_.step_limit_y));
  }

  double durationToLimit(const double position, const double velocity, const double limit) const
  {
    if (std::abs(velocity) < params_.velocity_epsilon) {
      return std::numeric_limits<double>::infinity();
    }

    const double target = std::copysign(limit, velocity);
    const double duration = (target - position) / velocity;
    if (!std::isfinite(duration)) {
      return std::numeric_limits<double>::infinity();
    }
    return std::max(0.0, duration);
  }

  void goToRestPoseCallback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    body_pose_mode_enabled_ = false;
    rest_sequence_active_ = true;
    rest_state_ = allLegsDown() ? chooseRestState() : RestState::LANDING;
    cmd_vel_ = zeroTwist();
    cmd_vel_smoothed_ = zeroTwist();
    body_pose_current_ = zeroTwist();
    cmd_vel_stamp_ = this->now();
    heading_hold_active_ = false;
    clearContinuousSwingState();
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

  KDL::Vector getRectangleBoundaryGoal(const KDL::Vector & direction) const
  {
    double scale = std::numeric_limits<double>::infinity();
    if (std::abs(direction.x()) >= params_.velocity_epsilon) {
      scale = std::min(scale, params_.step_limit_x / std::abs(direction.x()));
    }
    if (std::abs(direction.y()) >= params_.velocity_epsilon) {
      scale = std::min(scale, params_.step_limit_y / std::abs(direction.y()));
    }
    if (!std::isfinite(scale)) {
      return KDL::Vector(0.0, 0.0, 0.0);
    }
    return KDL::Vector(direction.x() * scale, direction.y() * scale, 0.0);
  }

  void moveSwingFootHorizontal(
    KDL::Vector & p,
    const KDL::Vector & foot_velocity,
    const double dt) const
  {
    const KDL::Vector goal_pos = -getRectangleBoundaryGoal(foot_velocity);
    const double dx = goal_pos.x() - p.x();
    const double dy = goal_pos.y() - p.y();
    const double distance = std::hypot(dx, dy);
    const double max_step = params_.swing_xy_velocity * dt;

    if (distance <= max_step || distance < params_.velocity_epsilon) {
      p.x(goal_pos.x());
      p.y(goal_pos.y());
    } else {
      const double ratio = max_step / distance;
      p.x(p.x() + dx * ratio);
      p.y(p.y() + dy * ratio);
    }
    clampFootXY(p);
  }

  bool footMotionActive(const std::vector<KDL::Vector> & foot_velocities) const
  {
    for (const KDL::Vector & foot_velocity : foot_velocities) {
      if (std::hypot(foot_velocity.x(), foot_velocity.y()) >= params_.velocity_epsilon) {
        return true;
      }
    }
    return false;
  }

  bool commandActive(const KDL::Twist & command) const
  {
    return std::hypot(command.vel.x(), command.vel.y()) >= params_.velocity_epsilon ||
      std::abs(command.rot.z()) >= params_.velocity_epsilon;
  }

  bool isTripodPhaseRepresentative(const bool left_tripod, const int leg_index) const
  {
    return left_tripod ? leg_index % 2 == 0 : leg_index % 2 == 1;
  }

  double tripodMaxOffset(const bool left_tripod) const
  {
    double max_offset = 0.0;
    for (int i = 0; i < nb_legs_; i++) {
      if (!isTripodPhaseRepresentative(left_tripod, i)) {
        continue;
      }
      const KDL::Vector & p = legs_[i].foot_relative_position;
      max_offset = std::max(max_offset, std::hypot(p.x(), p.y()));
    }
    return max_offset;
  }

  bool tripodNeedsRecentering(const bool left_tripod) const
  {
    return tripodMaxOffset(left_tripod) > params_.stop_recenter_tolerance;
  }

  bool allLegsDown() const
  {
    return left_phase_ == GaitPhase::DOWN && right_phase_ == GaitPhase::DOWN;
  }

  double wrappedPhase(double phase) const
  {
    phase = std::fmod(phase, 1.0);
    if (phase < 0.0) {
      phase += 1.0;
    }
    return phase;
  }

  double tripodPhase(const bool left_tripod) const
  {
    return wrappedPhase(gait_cycle_phase_ + (left_tripod ? 0.0 : 0.5));
  }

  GaitPhase phaseLabelFromTripodPhase(const double phase) const
  {
    if (phase >= swing_phase_fraction_) {
      return GaitPhase::DOWN;
    }

    const double swing_progress = phase / swing_phase_fraction_;
    return swing_progress < 0.5 ? GaitPhase::RISING : GaitPhase::FALLING;
  }

  void updatePhaseLabelsFromCycle()
  {
    left_phase_ = phaseLabelFromTripodPhase(tripodPhase(true));
    right_phase_ = phaseLabelFromTripodPhase(tripodPhase(false));
  }

  StopState chooseReplantState() const
  {
    const bool left_needs_recenter = tripodNeedsRecentering(true);
    const bool right_needs_recenter = tripodNeedsRecentering(false);
    if (!left_needs_recenter && !right_needs_recenter) {
      return StopState::HOLDING;
    }
    if (left_needs_recenter && !right_needs_recenter) {
      return StopState::REPLANT_LEFT;
    }
    if (!left_needs_recenter && right_needs_recenter) {
      return StopState::REPLANT_RIGHT;
    }
    return tripodMaxOffset(true) >= tripodMaxOffset(false) ?
      StopState::REPLANT_LEFT : StopState::REPLANT_RIGHT;
  }

  double tripodRestMaxError(const bool left_tripod) const
  {
    double max_error = 0.0;
    for (int i = 0; i < nb_legs_; i++) {
      if (!isTripodPhaseRepresentative(left_tripod, i)) {
        continue;
      }
      const KDL::Vector & p = legs_[i].foot_relative_position;
      const KDL::Vector & target = rest_pose_targets_[i];
      max_error = std::max(
        max_error,
        std::hypot(target.x() - p.x(), target.y() - p.y()));
    }
    return max_error;
  }

  bool tripodNeedsRestTarget(const bool left_tripod) const
  {
    return tripodRestMaxError(left_tripod) > params_.stop_recenter_tolerance;
  }

  bool allLegsAtRestPose() const
  {
    for (int i = 0; i < nb_legs_; i++) {
      const KDL::Vector & p = legs_[i].foot_relative_position;
      const KDL::Vector & target = rest_pose_targets_[i];
      if (std::hypot(target.x() - p.x(), target.y() - p.y()) > params_.stop_recenter_tolerance ||
        std::abs(target.z() - p.z()) > params_.velocity_epsilon)
      {
        return false;
      }
    }
    return true;
  }

  RestState chooseRestState() const
  {
    const bool left_needs_rest = tripodNeedsRestTarget(true);
    const bool right_needs_rest = tripodNeedsRestTarget(false);
    if (!left_needs_rest && !right_needs_rest) {
      return RestState::HOLDING;
    }
    if (left_needs_rest && !right_needs_rest) {
      return RestState::REPLANT_LEFT;
    }
    if (!left_needs_rest && right_needs_rest) {
      return RestState::REPLANT_RIGHT;
    }
    return tripodRestMaxError(true) >= tripodRestMaxError(false) ?
      RestState::REPLANT_LEFT : RestState::REPLANT_RIGHT;
  }

  KDL::Vector footVelocityForCommand(const int leg_index, const KDL::Twist & command) const
  {
    const KDL::Vector r =
      legs_[leg_index].foot_center_position + legs_[leg_index].foot_relative_position;
    return -KDL::Vector(
      command.vel.x() - command.rot.z() * r.y(),
      command.vel.y() + command.rot.z() * r.x(),
      0.0);
  }

  std::vector<KDL::Vector> computeFootVelocitiesForCommand(const KDL::Twist & command) const
  {
    std::vector<KDL::Vector> foot_velocities(nb_legs_);
    for (int i = 0; i < nb_legs_; i++) {
      foot_velocities[i] = footVelocityForCommand(i, command);
    }
    return foot_velocities;
  }

  double readyStanceError(
    const std::vector<KDL::Vector> & foot_velocities,
    const bool start_with_right) const
  {
    const bool swing_left_tripod = !start_with_right;
    double error = 0.0;
    for (int i = 0; i < nb_legs_; i++) {
      const KDL::Vector boundary = getRectangleBoundaryGoal(foot_velocities[i]);
      const bool swing_leg = isTripodPhaseRepresentative(swing_left_tripod, i);
      const KDL::Vector target = swing_leg ? boundary : -boundary;
      const KDL::Vector & p = legs_[i].foot_relative_position;
      error += std::hypot(target.x() - p.x(), target.y() - p.y());
    }
    return error;
  }

  bool chooseStartWithRightTripod(const KDL::Twist & command) const
  {
    const std::vector<KDL::Vector> foot_velocities = computeFootVelocitiesForCommand(command);
    const double right_start_error = readyStanceError(foot_velocities, true);
    const double left_start_error = readyStanceError(foot_velocities, false);
    if (std::abs(right_start_error - left_start_error) <= params_.velocity_epsilon) {
      return start_with_right_tripod_;
    }
    return right_start_error < left_start_error;
  }

  void setReadyStanceForCommand(const KDL::Twist & command, const bool start_with_right)
  {
    const std::vector<KDL::Vector> foot_velocities = computeFootVelocitiesForCommand(command);
    const bool swing_left_tripod = !start_with_right;
    for (int i = 0; i < nb_legs_; i++) {
      const KDL::Vector boundary = getRectangleBoundaryGoal(foot_velocities[i]);
      const bool swing_leg = isTripodPhaseRepresentative(swing_left_tripod, i);
      KDL::Vector & p = legs_[i].foot_relative_position;
      p.x(swing_leg ? boundary.x() : -boundary.x());
      p.y(swing_leg ? boundary.y() : -boundary.y());
      p.z(params_.foot_z_down);
      legs_[i].swing_start_position = p;
      legs_[i].in_continuous_swing = false;
    }
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

  KDL::Twist limitCommandRate(
    const KDL::Twist & current,
    const KDL::Twist & target,
    const double dt) const
  {
    KDL::Twist limited = current;
    const double linear_step = params_.max_linear_acceleration * dt;
    const double angular_step = params_.max_angular_acceleration * dt;

    limited.vel.x(moveTowards(current.vel.x(), target.vel.x(), linear_step));
    limited.vel.y(moveTowards(current.vel.y(), target.vel.y(), linear_step));
    limited.vel.z(moveTowards(current.vel.z(), target.vel.z(), linear_step));
    limited.rot.x(moveTowards(current.rot.x(), target.rot.x(), angular_step));
    limited.rot.y(moveTowards(current.rot.y(), target.rot.y(), angular_step));
    limited.rot.z(moveTowards(current.rot.z(), target.rot.z(), angular_step));
    return limited;
  }

  double clampSymmetric(const double value, const double limit) const
  {
    return std::clamp(value, -limit, limit);
  }

  KDL::Twist bodyPoseTargetFromCommand(const bool cmd_vel_timed_out) const
  {
    if (cmd_vel_timed_out) {
      return zeroTwist();
    }

    KDL::Twist target = zeroTwist();
    target.vel.x(clampSymmetric(cmd_vel_.vel.x(), params_.body_pose_max_x));
    target.vel.y(clampSymmetric(cmd_vel_.vel.y(), params_.body_pose_max_y));
    target.vel.z(clampSymmetric(cmd_vel_.vel.z(), params_.body_pose_max_z));
    target.rot.x(clampSymmetric(cmd_vel_.rot.x(), params_.body_pose_max_roll));
    target.rot.y(clampSymmetric(cmd_vel_.rot.y(), params_.body_pose_max_pitch));
    target.rot.z(clampSymmetric(cmd_vel_.rot.z(), params_.body_pose_max_yaw));
    return target;
  }

  KDL::Twist limitBodyPoseRate(
    const KDL::Twist & current,
    const KDL::Twist & target,
    const double dt) const
  {
    KDL::Twist limited = current;
    const double linear_step = params_.body_pose_linear_rate * dt;
    const double angular_step = params_.body_pose_angular_rate * dt;

    limited.vel.x(moveTowards(current.vel.x(), target.vel.x(), linear_step));
    limited.vel.y(moveTowards(current.vel.y(), target.vel.y(), linear_step));
    limited.vel.z(moveTowards(current.vel.z(), target.vel.z(), linear_step));
    limited.rot.x(moveTowards(current.rot.x(), target.rot.x(), angular_step));
    limited.rot.y(moveTowards(current.rot.y(), target.rot.y(), angular_step));
    limited.rot.z(moveTowards(current.rot.z(), target.rot.z(), angular_step));
    return limited;
  }

  void updateBodyPoseMode(const double dt, const bool cmd_vel_timed_out)
  {
    left_phase_ = GaitPhase::DOWN;
    right_phase_ = GaitPhase::DOWN;
    stop_state_ = StopState::HOLDING;
    cmd_vel_smoothed_ = zeroTwist();
    heading_hold_active_ = false;
    clearContinuousSwingState();

    const KDL::Twist target = bodyPoseTargetFromCommand(cmd_vel_timed_out);
    body_pose_current_ = limitBodyPoseRate(body_pose_current_, target, dt);

    const double roll = body_pose_current_.rot.x();
    const double pitch = body_pose_current_.rot.y();
    const double yaw = body_pose_current_.rot.z();
    const KDL::Vector translation = body_pose_current_.vel;

    const KDL::Rotation body_rotation = KDL::Rotation::RPY(roll, pitch, yaw);
    const KDL::Rotation base_from_world = body_rotation.Inverse();
    for (Leg & leg : legs_) {
      const KDL::Vector planted_foot_position =
        leg.foot_center_position + KDL::Vector(0.0, 0.0, params_.foot_z_down);
      const KDL::Vector foot_position_in_base =
        base_from_world * (planted_foot_position - translation);
      leg.foot_relative_position = foot_position_in_base - leg.foot_center_position;
    }
  }

  void holdCurrentPose()
  {
    left_phase_ = GaitPhase::DOWN;
    right_phase_ = GaitPhase::DOWN;
    clearContinuousSwingState();
    for (Leg & leg : legs_) {
      KDL::Vector & p = leg.foot_relative_position;
      p.z(params_.foot_z_down);
    }
  }

  void moveFootHorizontalToTarget(
    KDL::Vector & p,
    const double goal_x,
    const double goal_y,
    const double max_speed,
    const double dt) const
  {
    const double dx = goal_x - p.x();
    const double dy = goal_y - p.y();
    const double distance = std::hypot(dx, dy);
    const double max_step = max_speed * dt;

    if (distance <= max_step || distance < params_.velocity_epsilon) {
      p.x(goal_x);
      p.y(goal_y);
      return;
    }

    const double ratio = max_step / distance;
    p.x(p.x() + dx * ratio);
    p.y(p.y() + dy * ratio);
  }

  void landSwingFeetInPlace(const double dt)
  {
    for (int i = 0; i < nb_legs_; i++) {
      KDL::Vector & p = legs_[i].foot_relative_position;
      if (getLegPhase(i) == GaitPhase::DOWN) {
        legs_[i].in_continuous_swing = false;
        p.z(params_.foot_z_down);
        continue;
      }
      legs_[i].in_continuous_swing = false;
      p.z(std::max(params_.foot_z_down, p.z() - params_.vertical_velocity * dt));
    }

    if (left_phase_ != GaitPhase::DOWN &&
      legs_[0].foot_relative_position.z() <= params_.foot_z_down + params_.velocity_epsilon)
    {
      left_phase_ = GaitPhase::DOWN;
    }
    if (right_phase_ != GaitPhase::DOWN &&
      legs_[1].foot_relative_position.z() <= params_.foot_z_down + params_.velocity_epsilon)
    {
      right_phase_ = GaitPhase::DOWN;
    }
  }

  void replantTripod(const bool left_tripod, const double dt)
  {
    GaitPhase & active_phase = left_tripod ? left_phase_ : right_phase_;
    const int sample_leg_index = left_tripod ? 0 : 1;

    if (active_phase == GaitPhase::DOWN && tripodNeedsRecentering(left_tripod)) {
      active_phase = GaitPhase::RISING;
    }

    if (active_phase == GaitPhase::RISING &&
      legs_[sample_leg_index].foot_relative_position.z() + dt * params_.vertical_velocity >=
      params_.foot_z_up)
    {
      active_phase = GaitPhase::UP;
    }

    if (active_phase == GaitPhase::UP && !tripodNeedsRecentering(left_tripod)) {
      active_phase = GaitPhase::FALLING;
    }

    for (int i = 0; i < nb_legs_; i++) {
      KDL::Vector & p = legs_[i].foot_relative_position;
      if (!isTripodPhaseRepresentative(left_tripod, i)) {
        p.z(params_.foot_z_down);
        continue;
      }

      switch (active_phase) {
        case GaitPhase::DOWN:
          p.z(params_.foot_z_down);
          break;
        case GaitPhase::RISING:
          p.z(std::min(params_.foot_z_up, p.z() + params_.vertical_velocity * dt));
          moveFootHorizontalToTarget(p, 0.0, 0.0, params_.stop_recenter_velocity, dt);
          break;
        case GaitPhase::UP:
          p.z(params_.foot_z_up);
          moveFootHorizontalToTarget(p, 0.0, 0.0, params_.stop_recenter_velocity, dt);
          break;
        case GaitPhase::FALLING:
          p.z(std::max(params_.foot_z_down, p.z() - params_.vertical_velocity * dt));
          moveFootHorizontalToTarget(p, 0.0, 0.0, params_.stop_recenter_velocity, dt);
          break;
      }
    }

    if (active_phase == GaitPhase::UP && !tripodNeedsRecentering(left_tripod)) {
      active_phase = GaitPhase::FALLING;
    }

    if (active_phase == GaitPhase::FALLING &&
      legs_[sample_leg_index].foot_relative_position.z() <= params_.foot_z_down + params_.velocity_epsilon)
    {
      active_phase = GaitPhase::DOWN;
      for (int i = 0; i < nb_legs_; i++) {
        if (!isTripodPhaseRepresentative(left_tripod, i)) {
          continue;
        }
        legs_[i].foot_relative_position.z(params_.foot_z_down);
      }
    }
  }

  void replantTripodToRestTargets(const bool left_tripod, const double dt)
  {
    GaitPhase & active_phase = left_tripod ? left_phase_ : right_phase_;
    const int sample_leg_index = left_tripod ? 0 : 1;

    if (active_phase == GaitPhase::DOWN && tripodNeedsRestTarget(left_tripod)) {
      active_phase = GaitPhase::RISING;
    }

    if (active_phase == GaitPhase::RISING &&
      legs_[sample_leg_index].foot_relative_position.z() + dt * params_.vertical_velocity >=
      params_.foot_z_up)
    {
      active_phase = GaitPhase::UP;
    }

    if (active_phase == GaitPhase::UP && !tripodNeedsRestTarget(left_tripod)) {
      active_phase = GaitPhase::FALLING;
    }

    for (int i = 0; i < nb_legs_; i++) {
      KDL::Vector & p = legs_[i].foot_relative_position;
      if (!isTripodPhaseRepresentative(left_tripod, i)) {
        p.z(params_.foot_z_down);
        continue;
      }

      const KDL::Vector & target = rest_pose_targets_[i];
      switch (active_phase) {
        case GaitPhase::DOWN:
          p = target;
          break;
        case GaitPhase::RISING:
          p.z(std::min(params_.foot_z_up, p.z() + params_.vertical_velocity * dt));
          moveFootHorizontalToTarget(
            p, target.x(), target.y(), params_.stop_recenter_velocity, dt);
          break;
        case GaitPhase::UP:
          p.z(params_.foot_z_up);
          moveFootHorizontalToTarget(
            p, target.x(), target.y(), params_.stop_recenter_velocity, dt);
          break;
        case GaitPhase::FALLING:
          p.z(std::max(params_.foot_z_down, p.z() - params_.vertical_velocity * dt));
          moveFootHorizontalToTarget(
            p, target.x(), target.y(), params_.stop_recenter_velocity, dt);
          break;
      }
    }

    if (active_phase == GaitPhase::UP && !tripodNeedsRestTarget(left_tripod)) {
      active_phase = GaitPhase::FALLING;
    }

    if (active_phase == GaitPhase::FALLING &&
      legs_[sample_leg_index].foot_relative_position.z() <= params_.foot_z_down + params_.velocity_epsilon)
    {
      active_phase = GaitPhase::DOWN;
      for (int i = 0; i < nb_legs_; i++) {
        if (!isTripodPhaseRepresentative(left_tripod, i)) {
          continue;
        }
        legs_[i].foot_relative_position = rest_pose_targets_[i];
      }
    }
  }

  void beginStopSequence()
  {
    cmd_vel_smoothed_ = zeroTwist();
    heading_hold_active_ = false;
    clearContinuousSwingState();
    stop_state_ = allLegsDown() ? StopState::HOLDING : StopState::LANDING;
  }

  void beginWalkingSequence(const KDL::Twist & command)
  {
    rest_sequence_active_ = false;
    if (stop_state_ != StopState::WALKING && allLegsDown()) {
      const bool start_with_right = chooseStartWithRightTripod(command);
      if (params_.ready_stance_enabled && allLegsAtRestPose()) {
        setReadyStanceForCommand(command, start_with_right);
      }
      gait_cycle_phase_ = start_with_right ? 0.5 : 0.0;
      start_with_right_tripod_ = !start_with_right;
      updatePhaseLabelsFromCycle();
    }
    stop_state_ = StopState::WALKING;
  }

  void holdRestPose()
  {
    left_phase_ = GaitPhase::DOWN;
    right_phase_ = GaitPhase::DOWN;
    clearContinuousSwingState();
    for (int i = 0; i < nb_legs_; i++) {
      legs_[i].foot_relative_position = rest_pose_targets_[i];
    }
  }

  void updateRestSequence(const double dt)
  {
    cmd_vel_smoothed_ = zeroTwist();
    body_pose_current_ = zeroTwist();
    heading_hold_active_ = false;

    switch (rest_state_) {
      case RestState::LANDING:
        landSwingFeetInPlace(dt);
        if (allLegsDown()) {
          rest_state_ = chooseRestState();
        }
        break;
      case RestState::REPLANT_LEFT:
        replantTripodToRestTargets(true, dt);
        if (left_phase_ == GaitPhase::DOWN && !tripodNeedsRestTarget(true)) {
          rest_state_ = tripodNeedsRestTarget(false) ? RestState::REPLANT_RIGHT : RestState::HOLDING;
        }
        break;
      case RestState::REPLANT_RIGHT:
        replantTripodToRestTargets(false, dt);
        if (right_phase_ == GaitPhase::DOWN && !tripodNeedsRestTarget(false)) {
          rest_state_ = tripodNeedsRestTarget(true) ? RestState::REPLANT_LEFT : RestState::HOLDING;
        }
        break;
      case RestState::HOLDING:
        holdRestPose();
        rest_sequence_active_ = false;
        stop_state_ = StopState::HOLDING;
        break;
    }

    if (allLegsAtRestPose()) {
      holdRestPose();
      rest_sequence_active_ = false;
      rest_state_ = RestState::HOLDING;
      stop_state_ = StopState::HOLDING;
    }
  }

  void updateStopSequence(const double dt)
  {
    cmd_vel_smoothed_ = zeroTwist();
    heading_hold_active_ = false;

    switch (stop_state_) {
      case StopState::WALKING:
        beginStopSequence();
        break;
      case StopState::LANDING:
        landSwingFeetInPlace(dt);
        if (allLegsDown()) {
          stop_state_ = StopState::HOLDING;
        }
        break;
      case StopState::REPLANT_LEFT:
        replantTripod(true, dt);
        if (left_phase_ == GaitPhase::DOWN && !tripodNeedsRecentering(true)) {
          stop_state_ = tripodNeedsRecentering(false) ? StopState::REPLANT_RIGHT : StopState::HOLDING;
        }
        break;
      case StopState::REPLANT_RIGHT:
        replantTripod(false, dt);
        if (right_phase_ == GaitPhase::DOWN && !tripodNeedsRecentering(false)) {
          stop_state_ = tripodNeedsRecentering(true) ? StopState::REPLANT_LEFT : StopState::HOLDING;
        }
        break;
      case StopState::HOLDING:
        holdCurrentPose();
        break;
    }
  }

  std::vector<KDL::Vector> computeFootVelocities()
  {
    std::vector<KDL::Vector> foot_velocities(nb_legs_);
    double max_vel_ratio = 0.0;
    for (int i = 0; i < nb_legs_; i++) {
      const KDL::Twist & v = cmd_vel_smoothed_;
      const KDL::Vector r =
        legs_[i].foot_center_position + legs_[i].foot_relative_position;
      foot_velocities[i] = -KDL::Vector(
        v.vel.x() - v.rot.z() * r.y(), v.vel.y() + v.rot.z() * r.x(), 0.0);

      const double vel_ratio =
        std::hypot(
          foot_velocities[i].x() / params_.stance_velocity_x,
          foot_velocities[i].y() / params_.stance_velocity_y);
      max_vel_ratio = std::max(max_vel_ratio, vel_ratio);
    }

    if (max_vel_ratio > params_.max_velocity_ratio) {
      const double multiplier = params_.max_velocity_ratio / max_vel_ratio;
      cmd_vel_smoothed_ = cmd_vel_smoothed_ * multiplier;
      for (KDL::Vector & velocity : foot_velocities) {
        velocity = velocity * multiplier;
      }
    }
    return foot_velocities;
  }

  double computeGaitPhaseRate(const std::vector<KDL::Vector> & foot_velocities) const
  {
    double phase_rate = 0.0;
    for (const KDL::Vector & foot_velocity : foot_velocities) {
      if (std::abs(foot_velocity.x()) >= params_.velocity_epsilon) {
        phase_rate =
          std::max(phase_rate, std::abs(foot_velocity.x()) / (4.0 * params_.step_limit_x));
      }
      if (std::abs(foot_velocity.y()) >= params_.velocity_epsilon) {
        phase_rate =
          std::max(phase_rate, std::abs(foot_velocity.y()) / (4.0 * params_.step_limit_y));
      }
    }
    return phase_rate;
  }

  double smoothSwingHeight(const double swing_progress) const
  {
    return 0.5 * (1.0 - std::cos(2.0 * M_PI * std::clamp(swing_progress, 0.0, 1.0)));
  }

  double smoothSwingTravel(const double swing_progress) const
  {
    return std::clamp(swing_progress, 0.0, 1.0);
  }

  void clearContinuousSwingState()
  {
    for (Leg & leg : legs_) {
      leg.in_continuous_swing = false;
      leg.swing_start_position = leg.foot_relative_position;
    }
  }

  void moveLegsContinuous(
    const double dt,
    const std::vector<KDL::Vector> & foot_velocities)
  {
    for (int i = 0; i < nb_legs_; i++) {
      const bool left_tripod = i % 2 == 0;
      const double phase = tripodPhase(left_tripod);
      KDL::Vector & p = legs_[i].foot_relative_position;

      if (phase < swing_phase_fraction_) {
        const double swing_progress = phase / swing_phase_fraction_;
        const double lift = smoothSwingHeight(swing_progress);
        const double travel = smoothSwingTravel(swing_progress);
        if (!legs_[i].in_continuous_swing) {
          legs_[i].in_continuous_swing = true;
          legs_[i].swing_start_position = p;
        }
        const KDL::Vector goal = -getRectangleBoundaryGoal(foot_velocities[i]);
        p.x(legs_[i].swing_start_position.x() +
          (goal.x() - legs_[i].swing_start_position.x()) * travel);
        p.y(legs_[i].swing_start_position.y() +
          (goal.y() - legs_[i].swing_start_position.y()) * travel);
        p.z(params_.foot_z_down + (params_.foot_z_up - params_.foot_z_down) * lift);
        clampFootXY(p);
      } else {
        legs_[i].in_continuous_swing = false;
        p.z(params_.foot_z_down);
        p += foot_velocities[i] * dt;
        clampFootXY(p);
      }
    }
  }

  void updateContinuousGait(
    const double dt,
    const std::vector<KDL::Vector> & foot_velocities)
  {
    gait_cycle_phase_ = wrappedPhase(gait_cycle_phase_ + computeGaitPhaseRate(foot_velocities) * dt);
    updatePhaseLabelsFromCycle();
    moveLegsContinuous(dt, foot_velocities);
  }

  double getMinimumSupportDuration(const std::vector<KDL::Vector> & foot_velocities)
  {
    double min_duration_to_rising = std::numeric_limits<double>::infinity();
    for (int i = 0; i < nb_legs_; i++) {
      if (getLegPhase(i) != GaitPhase::DOWN) {
        continue;
      }
      const KDL::Vector & p = legs_[i].foot_relative_position;
      const KDL::Vector & foot_velocity = foot_velocities[i];
      const double duration_to_rising_this_leg = std::min(
        durationToLimit(p.x(), foot_velocity.x(), params_.step_limit_x),
        durationToLimit(p.y(), foot_velocity.y(), params_.step_limit_y));
      min_duration_to_rising =
        std::min(min_duration_to_rising, duration_to_rising_this_leg);
    }
    return min_duration_to_rising;
  }

  void updatePhases(
    const double dt,
    const double min_duration_to_rising,
    const double duration_vertical)
  {
    if (left_phase_ == GaitPhase::DOWN && right_phase_ == GaitPhase::DOWN) {
      if (min_duration_to_rising <= dt) {
        right_phase_ = GaitPhase::RISING;
      }
    } else if (left_phase_ == GaitPhase::DOWN &&
      right_phase_ == GaitPhase::FALLING)
    {
      if (min_duration_to_rising <= dt &&
        legs_[1].foot_relative_position.z() <= params_.foot_z_down + params_.velocity_epsilon)
      {
        left_phase_ = GaitPhase::RISING;
        right_phase_ = GaitPhase::DOWN;
      }
    } else if (right_phase_ == GaitPhase::DOWN &&
      left_phase_ == GaitPhase::FALLING)
    {
      if (min_duration_to_rising <= dt &&
        legs_[0].foot_relative_position.z() <= params_.foot_z_down + params_.velocity_epsilon)
      {
        left_phase_ = GaitPhase::DOWN;
        right_phase_ = GaitPhase::RISING;
      }
    } else if (left_phase_ == GaitPhase::DOWN && right_phase_ == GaitPhase::UP) {
      if (min_duration_to_rising <= duration_vertical + dt) {
        right_phase_ = GaitPhase::FALLING;
      }
    } else if (right_phase_ == GaitPhase::DOWN && left_phase_ == GaitPhase::UP) {
      if (min_duration_to_rising <= duration_vertical + dt) {
        left_phase_ = GaitPhase::FALLING;
      }
    }

    if (left_phase_ == GaitPhase::RISING &&
      legs_[0].foot_relative_position.z() + dt * params_.vertical_velocity >= params_.foot_z_up)
    {
      left_phase_ = GaitPhase::UP;
    }
    if (right_phase_ == GaitPhase::RISING &&
      legs_[1].foot_relative_position.z() + dt * params_.vertical_velocity >= params_.foot_z_up)
    {
      right_phase_ = GaitPhase::UP;
    }
  }

  void moveLegs(
    const double dt,
    const std::vector<KDL::Vector> & foot_velocities)
  {
    for (int i = 0; i < nb_legs_; i++) {
      KDL::Vector & p = legs_[i].foot_relative_position;
      switch (getLegPhase(i)) {
        case GaitPhase::DOWN:
          p.z(params_.foot_z_down);
          p += foot_velocities[i] * dt;
          clampFootXY(p);
          break;
        case GaitPhase::RISING:
          p.z(std::min(params_.foot_z_up, p.z() + params_.vertical_velocity * dt));
          moveSwingFootHorizontal(p, foot_velocities[i], dt);
          break;
        case GaitPhase::UP:
          p.z(params_.foot_z_up);
          moveSwingFootHorizontal(p, foot_velocities[i], dt);
          break;
        case GaitPhase::FALLING:
          p.z(std::max(params_.foot_z_down, p.z() - params_.vertical_velocity * dt));
          if (p.z() < params_.foot_z_sync) {
            p += foot_velocities[i] * dt;
            clampFootXY(p);
          } else {
            moveSwingFootHorizontal(p, foot_velocities[i], dt);
          }
          break;
      }
    }
  }

  void timerCallback()
  {
    const rclcpp::Time now = this->now();
    const double dt = getDt();
    const double cmd_vel_oldness = (now - cmd_vel_stamp_).seconds();
    const bool cmd_vel_timed_out = cmd_vel_oldness > params_.cmd_vel_timeout;

    const KDL::Twist raw_target_command = cmd_vel_timed_out ? zeroTwist() : cmd_vel_;
    const bool interrupt_rest = commandActive(raw_target_command);

    if (rest_sequence_active_ && !interrupt_rest) {
      updateRestSequence(dt);
    } else if (body_pose_mode_enabled_) {
      rest_sequence_active_ = false;
      updateBodyPoseMode(dt, cmd_vel_timed_out);
    } else {
      rest_sequence_active_ = false;
      if (cmd_vel_timed_out) {
        RCLCPP_INFO_THROTTLE(
          this->get_logger(), *this->get_clock(), 2000,
          "Received cmd_vel timed out (%.2f > %.2f s), holding stance.",
          cmd_vel_oldness, params_.cmd_vel_timeout);
      }

      const KDL::Twist target_command =
        cmd_vel_timed_out ? zeroTwist() : applyHeadingHold(cmd_vel_);
      const bool motion_command_active = commandActive(target_command);

      if (!motion_command_active) {
        if (stop_state_ == StopState::WALKING) {
          beginStopSequence();
        }
        updateStopSequence(dt);
      } else {
        if (stop_state_ != StopState::WALKING && allLegsDown()) {
          beginWalkingSequence(target_command);
        } else {
          stop_state_ = StopState::WALKING;
        }

        cmd_vel_smoothed_ = limitCommandRate(cmd_vel_smoothed_, target_command, dt);

        std::vector<KDL::Vector> foot_velocities = computeFootVelocities();
        const bool active_motion = footMotionActive(foot_velocities);

        if (!active_motion) {
          holdCurrentPose();
        } else {
          updateContinuousGait(dt, foot_velocities);
        }
      }
    }

  // Get joint angles (inverse kinematics)
  for (int i = 0; i < nb_legs_; i++) {
    const KDL::Vector foot_position =
      legs_[i].foot_center_position + legs_[i].foot_relative_position;
    KDL::Frame foot_frame(foot_position);
    KDL::JntArray joint_angles_next = legs_[i].joint_angles;
    const int ik_result = kinematics_->cartToJnt(
      i, legs_[i].joint_angles,
      foot_frame,
      joint_angles_next);
    if (ik_result >= 0) {
      kinematics_->foldAndClampJointAnglesToLimits(i, joint_angles_next);
      legs_[i].joint_angles = joint_angles_next;
    } else {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 1000,
        "IK failed for leg %d, keeping previous joint command.", i);
    }
  }

  // Publish actuator setpoints
  actuator_msgs::msg::Actuators msg;
  msg.header.stamp = now;
  msg.position.resize(nb_legs_*nb_joints_per_leg_);
  std_msgs::msg::Float64 joint_position_msg;
  for (int i = 0; i < nb_legs_; i++) {
    for (int j = 0; j < nb_joints_per_leg_; j++) {
      const size_t joint_index = i * nb_joints_per_leg_ + j;
      msg.position[joint_index] = legs_[i].joint_angles(j);
      joint_position_msg.data = msg.position[joint_index];
      joint_position_pubs_[joint_index]->publish(joint_position_msg);
    }
  }
  actuators_pub_->publish(msg);

  // Adjusted velocity (for odometry and debug)
  {
    geometry_msgs::msg::TwistStamped msg;
    msg.header.stamp = now;
    msg.twist.linear.x = cmd_vel_smoothed_.vel.x();
    msg.twist.linear.y = cmd_vel_smoothed_.vel.y();
    msg.twist.angular.z = cmd_vel_smoothed_.rot.z();
    cmd_vel_adj_pub_->publish(msg);
  }

  // For debug, also publish relative foot cartesian positions
  antsy_msgs::msg::Vector3Array foot_positions_msg;
  foot_positions_msg.header.stamp = now;
  foot_positions_msg.vectors.resize(nb_legs_);
  for (int i = 0; i < nb_legs_; i++) {
    foot_positions_msg.vectors[i].x = legs_[i].foot_relative_position.x();
    foot_positions_msg.vectors[i].y = legs_[i].foot_relative_position.y();
    foot_positions_msg.vectors[i].z = legs_[i].foot_relative_position.z();
  }
  relative_foot_position_pub_->publish(foot_positions_msg);

  // and publish gait phases
  antsy_msgs::msg::GaitPhase phase_left_msg;
  phase_left_msg.header.stamp = now;
  phase_left_msg.phase = static_cast<uint8_t>(left_phase_);
  gait_phase_left_pub_->publish(phase_left_msg);
  antsy_msgs::msg::GaitPhase phase_right_msg;
  phase_right_msg.header.stamp = now;
  phase_right_msg.phase = static_cast<uint8_t>(right_phase_);
  gait_phase_right_pub_->publish(phase_right_msg);
}

  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr
    cmd_vel_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr
    cmd_vel_unstamped_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr
    odom_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr
    body_pose_mode_sub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr
    go_to_rest_pose_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr
    reset_control_srv_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr
    robot_description_sub_;
  rclcpp::Publisher<actuator_msgs::msg::Actuators>::SharedPtr
    actuators_pub_;
  std::vector<rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr>
    joint_position_pubs_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr
    cmd_vel_adj_pub_;
  rclcpp::Publisher<antsy_msgs::msg::Vector3Array>::SharedPtr
    relative_foot_position_pub_;
  rclcpp::Publisher<antsy_msgs::msg::GaitPhase>::SharedPtr
    gait_phase_left_pub_;
  rclcpp::Publisher<antsy_msgs::msg::GaitPhase>::SharedPtr
    gait_phase_right_pub_;

  rclcpp::TimerBase::SharedPtr timer_;
  std::shared_ptr<antsy_kinematics::Kinematics> kinematics_;

  rclcpp::Clock steady_clock_{RCL_STEADY_TIME};
  rclcpp::Time last_update_{0, 0, RCL_STEADY_TIME};
  std::vector<Leg> legs_;
  KDL::Twist cmd_vel_;
  KDL::Twist cmd_vel_smoothed_;
  rclcpp::Time cmd_vel_stamp_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
  bool has_odom_ = false;
  double current_yaw_ = 0.0;
  double current_yaw_rate_ = 0.0;
  bool heading_hold_active_ = false;
  double heading_hold_yaw_ = 0.0;
  bool body_pose_mode_enabled_ = false;
  KDL::Twist body_pose_current_;
  StopState stop_state_ = StopState::HOLDING;
  bool rest_sequence_active_ = false;
  RestState rest_state_ = RestState::HOLDING;
  std::vector<KDL::Vector> rest_pose_targets_;
  double gait_cycle_phase_ = 0.0;
  bool start_with_right_tripod_ = true;
  // Phases of the two front feet
  // alternating feet are in phase
  GaitPhase left_phase_;
  GaitPhase right_phase_;
  Parameters params_;

  // TODO support other values than 6
  static const int nb_legs_ = 6;
  static const int nb_joints_per_leg_ = 3;
  static constexpr double swing_phase_fraction_ = 0.5;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<FollowVelocity>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
