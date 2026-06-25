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
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/string.hpp"
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

struct Leg {
  // neutral position, expressed in robot base_link
  KDL::Vector foot_center_position;
  // last foot position, expressed relative to neutral position
  KDL::Vector foot_relative_position;
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

    // Listen to velocity setpoint
    cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
      "cmd_vel", 10, std::bind(&FollowVelocity::cmdVelCallback, this, _1));
    cmd_vel_unstamped_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel_unstamped", 10, std::bind(&FollowVelocity::cmdVelUnstampedCallback, this, _1));
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
      legs_[i].joint_angles = KDL::JntArray(nb_joints_per_leg_);
      const std::vector<double> & seed =
        i < nb_legs_ / 2 ? params_.ik_seed_joint_angles_left : params_.ik_seed_joint_angles_right;
      for (int j = 0; j < nb_joints_per_leg_; j++) {
        legs_[i].joint_angles(j) = seed[j];
      }
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
    double ik_position_weight;
    double ik_orientation_weight;
    std::vector<double> ik_seed_joint_angles_left;
    std::vector<double> ik_seed_joint_angles_right;
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
      params_.ik_position_weight <= 0.0 ||
      params_.ik_orientation_weight < 0.0 ||
      params_.ik_seed_joint_angles_left.size() != nb_joints_per_leg_ ||
      params_.ik_seed_joint_angles_right.size() != nb_joints_per_leg_ ||
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

  KDL::Twist zeroTwist() const
  {
    return KDL::Twist(KDL::Vector(0.0, 0.0, 0.0), KDL::Vector(0.0, 0.0, 0.0));
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

  void standStill(const double dt)
  {
    left_phase_ = GaitPhase::DOWN;
    right_phase_ = GaitPhase::DOWN;
    for (Leg & leg : legs_) {
      KDL::Vector & p = leg.foot_relative_position;
      const double xy_step = params_.idle_return_velocity * dt;
      p.x(moveTowards(p.x(), 0.0, xy_step));
      p.y(moveTowards(p.y(), 0.0, xy_step));
      p.z(moveTowards(p.z(), params_.foot_z_down, params_.vertical_velocity * dt));
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
    if (cmd_vel_timed_out) {
      RCLCPP_INFO_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "Received cmd_vel timed out (%.2f > %.2f s), holding stance.",
        cmd_vel_oldness, params_.cmd_vel_timeout);
    }

    const bool falling =
      left_phase_ == GaitPhase::FALLING || right_phase_ == GaitPhase::FALLING;
    if (!falling) {
      cmd_vel_smoothed_ = cmd_vel_timed_out ? zeroTwist() : cmd_vel_;
    }

    std::vector<KDL::Vector> foot_velocities = computeFootVelocities();
    const bool active_motion = footMotionActive(foot_velocities);

    if (!active_motion) {
      standStill(dt);
    } else {
      const double duration_vertical =
        (params_.foot_z_up - params_.foot_z_down) / params_.vertical_velocity;
      const double min_duration_to_rising = getMinimumSupportDuration(foot_velocities);
      updatePhases(dt, min_duration_to_rising, duration_vertical);
      moveLegs(dt, foot_velocities);
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
  // Phases of the two front feet
  // alternating feet are in phase
  GaitPhase left_phase_;
  GaitPhase right_phase_;
  Parameters params_;

  // TODO support other values than 6
  static const int nb_legs_ = 6;
  static const int nb_joints_per_leg_ = 3;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<FollowVelocity>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
