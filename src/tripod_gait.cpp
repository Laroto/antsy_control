#include "antsy_control/tripod_gait.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace antsy_control
{

KDL::Twist zeroTwist()
{
  return KDL::Twist(KDL::Vector(0.0, 0.0, 0.0), KDL::Vector(0.0, 0.0, 0.0));
}

TripodGait::TripodGait(const TripodGaitParameters & params)
: params_(params),
  cmd_vel_smoothed_(zeroTwist()),
  body_pose_current_(zeroTwist())
{
  validateParameters();
  initializeLegs();
  reset();
}

void TripodGait::validateParameters() const
{
  if (
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
    throw std::runtime_error("Invalid hexapod gait parameters.");
  }
}

void TripodGait::initializeLegs()
{
  legs_.resize(kNumLegs);
  for (int i = 0; i < kNumLegs; i++) {
    legs_[i].foot_center_position.x(
      (i == 0 || i == 5 ? 0.18 : 0.00) + (i == 2 || i == 3 ? -0.18 : 0.00));
    legs_[i].foot_center_position.y(
      (i == 1 || i == 4 ? 0.27 : 0.20) * (i < 3 ? 1 : -1));
    legs_[i].foot_center_position.z(0.0);
    legs_[i].foot_relative_position = KDL::Vector(0.0, 0.0, params_.foot_z_down);
    legs_[i].swing_start_position = legs_[i].foot_relative_position;
    legs_[i].swing_goal_position = legs_[i].foot_relative_position;
    legs_[i].in_continuous_swing = false;
    legs_[i].joint_angles = KDL::JntArray(kNumJointsPerLeg);
  }

  rest_pose_targets_.resize(kNumLegs);
  for (int i = 0; i < kNumLegs; i++) {
    rest_pose_targets_[i] = legs_[i].foot_relative_position;
  }
}

void TripodGait::reset()
{
  cmd_vel_smoothed_ = zeroTwist();
  body_pose_current_ = zeroTwist();
  body_pose_mode_enabled_ = false;
  stop_state_ = StopState::HOLDING;
  rest_sequence_active_ = false;
  rest_state_ = RestState::HOLDING;
  gait_cycle_phase_ = 0.0;
  start_with_right_tripod_ = true;
  left_phase_ = GaitPhase::DOWN;
  right_phase_ = GaitPhase::DOWN;

  for (int i = 0; i < kNumLegs; i++) {
    legs_[i].foot_relative_position = rest_pose_targets_[i];
    legs_[i].swing_start_position = legs_[i].foot_relative_position;
    legs_[i].swing_goal_position = legs_[i].foot_relative_position;
    legs_[i].in_continuous_swing = false;
  }
}

void TripodGait::setBodyPoseModeEnabled(const bool enabled)
{
  if (body_pose_mode_enabled_ == enabled) {
    return;
  }

  body_pose_mode_enabled_ = enabled;
  rest_sequence_active_ = false;
  cmd_vel_smoothed_ = zeroTwist();
  body_pose_current_ = zeroTwist();
  clearContinuousSwingState();
  left_phase_ = GaitPhase::DOWN;
  right_phase_ = GaitPhase::DOWN;
  stop_state_ = StopState::HOLDING;
}

bool TripodGait::bodyPoseModeEnabled() const
{
  return body_pose_mode_enabled_;
}

void TripodGait::beginRestSequence()
{
  body_pose_mode_enabled_ = false;
  rest_sequence_active_ = true;
  rest_state_ = allLegsDown() ? chooseRestState() : RestState::LANDING;
  cmd_vel_smoothed_ = zeroTwist();
  body_pose_current_ = zeroTwist();
  clearContinuousSwingState();
}

void TripodGait::update(
  const double dt,
  const KDL::Twist & command,
  const bool command_timed_out)
{
  const KDL::Twist raw_target_command = command_timed_out ? zeroTwist() : command;
  const bool interrupt_rest = commandActive(raw_target_command);

  if (rest_sequence_active_ && !interrupt_rest) {
    updateRestSequence(dt);
  } else if (body_pose_mode_enabled_) {
    rest_sequence_active_ = false;
    updateBodyPoseMode(dt, command, command_timed_out);
  } else {
    rest_sequence_active_ = false;
    const KDL::Twist target_command = command_timed_out ? zeroTwist() : command;
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
      if (!footMotionActive(foot_velocities)) {
        holdCurrentPose();
      } else {
        updateContinuousGait(dt, foot_velocities);
      }
    }
  }
}

const std::vector<Leg> & TripodGait::legs() const
{
  return legs_;
}

std::vector<Leg> & TripodGait::mutableLegs()
{
  return legs_;
}

const KDL::Twist & TripodGait::smoothedCommand() const
{
  return cmd_vel_smoothed_;
}

GaitPhase TripodGait::phaseForLeg(const int leg_index) const
{
  return leg_index % 2 ? right_phase_ : left_phase_;
}

bool TripodGait::supportForLeg(const int leg_index) const
{
  return phaseForLeg(leg_index) == GaitPhase::DOWN;
}

GaitPhase TripodGait::leftPhase() const
{
  return left_phase_;
}

GaitPhase TripodGait::rightPhase() const
{
  return right_phase_;
}

bool TripodGait::allLegsDown() const
{
  return left_phase_ == GaitPhase::DOWN && right_phase_ == GaitPhase::DOWN;
}

void TripodGait::clearContinuousSwingState()
{
  for (Leg & leg : legs_) {
    leg.in_continuous_swing = false;
    leg.swing_start_position = leg.foot_relative_position;
    leg.swing_goal_position = leg.foot_relative_position;
  }
}

bool TripodGait::commandActive(const KDL::Twist & command) const
{
  return std::hypot(command.vel.x(), command.vel.y()) >= params_.velocity_epsilon ||
    std::abs(command.rot.z()) >= params_.velocity_epsilon;
}

bool TripodGait::footMotionActive(const std::vector<KDL::Vector> & foot_velocities) const
{
  for (const KDL::Vector & foot_velocity : foot_velocities) {
    if (std::hypot(foot_velocity.x(), foot_velocity.y()) >= params_.velocity_epsilon) {
      return true;
    }
  }
  return false;
}

bool TripodGait::isTripodPhaseRepresentative(
  const bool left_tripod,
  const int leg_index) const
{
  return left_tripod ? leg_index % 2 == 0 : leg_index % 2 == 1;
}

void TripodGait::clampFootXY(KDL::Vector & p) const
{
  p.x(std::clamp(p.x(), -params_.step_limit_x, params_.step_limit_x));
  p.y(std::clamp(p.y(), -params_.step_limit_y, params_.step_limit_y));
}

double TripodGait::moveTowards(
  const double current,
  const double target,
  const double max_delta) const
{
  const double delta = target - current;
  if (std::abs(delta) <= max_delta) {
    return target;
  }
  return current + std::copysign(max_delta, delta);
}

double TripodGait::clampSymmetric(const double value, const double limit) const
{
  return std::clamp(value, -limit, limit);
}

double TripodGait::wrappedPhase(double phase) const
{
  phase = std::fmod(phase, 1.0);
  if (phase < 0.0) {
    phase += 1.0;
  }
  return phase;
}

double TripodGait::tripodPhase(const bool left_tripod) const
{
  return wrappedPhase(gait_cycle_phase_ + (left_tripod ? 0.0 : 0.5));
}

GaitPhase TripodGait::phaseLabelFromTripodPhase(const double phase) const
{
  if (phase >= swing_phase_fraction_) {
    return GaitPhase::DOWN;
  }

  const double swing_progress = phase / swing_phase_fraction_;
  return swing_progress < 0.5 ? GaitPhase::RISING : GaitPhase::FALLING;
}

void TripodGait::updatePhaseLabelsFromCycle()
{
  left_phase_ = phaseLabelFromTripodPhase(tripodPhase(true));
  right_phase_ = phaseLabelFromTripodPhase(tripodPhase(false));
}

double TripodGait::durationToLimit(
  const double position,
  const double velocity,
  const double limit) const
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

KDL::Vector TripodGait::getRectangleBoundaryGoal(const KDL::Vector & direction) const
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

void TripodGait::moveSwingFootHorizontal(
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

void TripodGait::moveFootHorizontalToTarget(
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

double TripodGait::tripodMaxOffset(const bool left_tripod) const
{
  double max_offset = 0.0;
  for (int i = 0; i < kNumLegs; i++) {
    if (!isTripodPhaseRepresentative(left_tripod, i)) {
      continue;
    }
    const KDL::Vector & p = legs_[i].foot_relative_position;
    max_offset = std::max(max_offset, std::hypot(p.x(), p.y()));
  }
  return max_offset;
}

bool TripodGait::tripodNeedsRecentering(const bool left_tripod) const
{
  return tripodMaxOffset(left_tripod) > params_.stop_recenter_tolerance;
}

TripodGait::StopState TripodGait::chooseReplantState() const
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

double TripodGait::tripodRestMaxError(const bool left_tripod) const
{
  double max_error = 0.0;
  for (int i = 0; i < kNumLegs; i++) {
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

bool TripodGait::tripodNeedsRestTarget(const bool left_tripod) const
{
  return tripodRestMaxError(left_tripod) > params_.stop_recenter_tolerance;
}

bool TripodGait::allLegsAtRestPose() const
{
  for (int i = 0; i < kNumLegs; i++) {
    const KDL::Vector & p = legs_[i].foot_relative_position;
    const KDL::Vector & target = rest_pose_targets_[i];
    if (
      std::hypot(target.x() - p.x(), target.y() - p.y()) >
      params_.stop_recenter_tolerance ||
      std::abs(target.z() - p.z()) > params_.velocity_epsilon)
    {
      return false;
    }
  }
  return true;
}

TripodGait::RestState TripodGait::chooseRestState() const
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

KDL::Vector TripodGait::footVelocityForCommand(
  const int leg_index,
  const KDL::Twist & command) const
{
  const KDL::Vector r =
    legs_[leg_index].foot_center_position + legs_[leg_index].foot_relative_position;
  return -KDL::Vector(
    command.vel.x() - command.rot.z() * r.y(),
    command.vel.y() + command.rot.z() * r.x(),
    0.0);
}

std::vector<KDL::Vector> TripodGait::computeFootVelocitiesForCommand(
  const KDL::Twist & command) const
{
  std::vector<KDL::Vector> foot_velocities(kNumLegs);
  for (int i = 0; i < kNumLegs; i++) {
    foot_velocities[i] = footVelocityForCommand(i, command);
  }
  return foot_velocities;
}

double TripodGait::readyStanceError(
  const std::vector<KDL::Vector> & foot_velocities,
  const bool start_with_right) const
{
  const bool swing_left_tripod = !start_with_right;
  double error = 0.0;
  for (int i = 0; i < kNumLegs; i++) {
    const KDL::Vector boundary = getRectangleBoundaryGoal(foot_velocities[i]);
    const bool swing_leg = isTripodPhaseRepresentative(swing_left_tripod, i);
    const KDL::Vector target = swing_leg ? boundary : -boundary;
    const KDL::Vector & p = legs_[i].foot_relative_position;
    error += std::hypot(target.x() - p.x(), target.y() - p.y());
  }
  return error;
}

bool TripodGait::chooseStartWithRightTripod(const KDL::Twist & command) const
{
  const std::vector<KDL::Vector> foot_velocities = computeFootVelocitiesForCommand(command);
  const double right_start_error = readyStanceError(foot_velocities, true);
  const double left_start_error = readyStanceError(foot_velocities, false);
  if (std::abs(right_start_error - left_start_error) <= params_.velocity_epsilon) {
    return start_with_right_tripod_;
  }
  return right_start_error < left_start_error;
}

bool TripodGait::readyStanceApplies(const KDL::Twist & command) const
{
  const double ready_norm = std::hypot(
    std::hypot(params_.ready_stance_linear_x, params_.ready_stance_linear_y),
    params_.ready_stance_angular_z);
  const double command_norm = std::hypot(
    std::hypot(command.vel.x(), command.vel.y()),
    command.rot.z());
  if (ready_norm < params_.velocity_epsilon || command_norm < params_.velocity_epsilon) {
    return false;
  }

  const double dot =
    command.vel.x() * params_.ready_stance_linear_x +
    command.vel.y() * params_.ready_stance_linear_y +
    command.rot.z() * params_.ready_stance_angular_z;
  return dot / (ready_norm * command_norm) > 0.707;
}

void TripodGait::setReadyStanceForCommand(
  const KDL::Twist & command,
  const bool start_with_right)
{
  const std::vector<KDL::Vector> foot_velocities = computeFootVelocitiesForCommand(command);
  const bool swing_left_tripod = !start_with_right;
  for (int i = 0; i < kNumLegs; i++) {
    const KDL::Vector boundary = getRectangleBoundaryGoal(foot_velocities[i]);
    const bool swing_leg = isTripodPhaseRepresentative(swing_left_tripod, i);
    KDL::Vector & p = legs_[i].foot_relative_position;
    p.x(swing_leg ? boundary.x() : -boundary.x());
    p.y(swing_leg ? boundary.y() : -boundary.y());
    p.z(params_.foot_z_down);
    legs_[i].swing_start_position = p;
    legs_[i].swing_goal_position = p;
    legs_[i].in_continuous_swing = false;
  }
}

KDL::Twist TripodGait::limitCommandRate(
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

KDL::Twist TripodGait::bodyPoseTargetFromCommand(
  const KDL::Twist & command,
  const bool command_timed_out) const
{
  if (command_timed_out) {
    return zeroTwist();
  }

  KDL::Twist target = zeroTwist();
  target.vel.x(clampSymmetric(command.vel.x(), params_.body_pose_max_x));
  target.vel.y(clampSymmetric(command.vel.y(), params_.body_pose_max_y));
  target.vel.z(clampSymmetric(command.vel.z(), params_.body_pose_max_z));
  target.rot.x(clampSymmetric(command.rot.x(), params_.body_pose_max_roll));
  target.rot.y(clampSymmetric(command.rot.y(), params_.body_pose_max_pitch));
  target.rot.z(clampSymmetric(command.rot.z(), params_.body_pose_max_yaw));
  return target;
}

KDL::Twist TripodGait::limitBodyPoseRate(
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

void TripodGait::updateBodyPoseMode(
  const double dt,
  const KDL::Twist & command,
  const bool command_timed_out)
{
  left_phase_ = GaitPhase::DOWN;
  right_phase_ = GaitPhase::DOWN;
  stop_state_ = StopState::HOLDING;
  cmd_vel_smoothed_ = zeroTwist();
  clearContinuousSwingState();

  const KDL::Twist target = bodyPoseTargetFromCommand(command, command_timed_out);
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

void TripodGait::holdCurrentPose()
{
  left_phase_ = GaitPhase::DOWN;
  right_phase_ = GaitPhase::DOWN;
  clearContinuousSwingState();
  for (Leg & leg : legs_) {
    KDL::Vector & p = leg.foot_relative_position;
    p.z(params_.foot_z_down);
  }
}

void TripodGait::landSwingFeetInPlace(const double dt)
{
  for (int i = 0; i < kNumLegs; i++) {
    KDL::Vector & p = legs_[i].foot_relative_position;
    if (phaseForLeg(i) == GaitPhase::DOWN) {
      legs_[i].in_continuous_swing = false;
      p.z(params_.foot_z_down);
      continue;
    }
    legs_[i].in_continuous_swing = false;
    p.z(std::max(params_.foot_z_down, p.z() - params_.vertical_velocity * dt));
  }

  if (
    left_phase_ != GaitPhase::DOWN &&
    legs_[0].foot_relative_position.z() <= params_.foot_z_down + params_.velocity_epsilon)
  {
    left_phase_ = GaitPhase::DOWN;
  }
  if (
    right_phase_ != GaitPhase::DOWN &&
    legs_[1].foot_relative_position.z() <= params_.foot_z_down + params_.velocity_epsilon)
  {
    right_phase_ = GaitPhase::DOWN;
  }
}

void TripodGait::replantTripod(const bool left_tripod, const double dt)
{
  GaitPhase & active_phase = left_tripod ? left_phase_ : right_phase_;
  const int sample_leg_index = left_tripod ? 0 : 1;

  if (active_phase == GaitPhase::DOWN && tripodNeedsRecentering(left_tripod)) {
    active_phase = GaitPhase::RISING;
  }

  if (
    active_phase == GaitPhase::RISING &&
    legs_[sample_leg_index].foot_relative_position.z() + dt * params_.vertical_velocity >=
    params_.foot_z_up)
  {
    active_phase = GaitPhase::UP;
  }

  if (active_phase == GaitPhase::UP && !tripodNeedsRecentering(left_tripod)) {
    active_phase = GaitPhase::FALLING;
  }

  for (int i = 0; i < kNumLegs; i++) {
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

  if (
    active_phase == GaitPhase::FALLING &&
    legs_[sample_leg_index].foot_relative_position.z() <= params_.foot_z_down + params_.velocity_epsilon)
  {
    active_phase = GaitPhase::DOWN;
    for (int i = 0; i < kNumLegs; i++) {
      if (isTripodPhaseRepresentative(left_tripod, i)) {
        legs_[i].foot_relative_position.z(params_.foot_z_down);
      }
    }
  }
}

void TripodGait::replantTripodToRestTargets(const bool left_tripod, const double dt)
{
  GaitPhase & active_phase = left_tripod ? left_phase_ : right_phase_;
  const int sample_leg_index = left_tripod ? 0 : 1;

  if (active_phase == GaitPhase::DOWN && tripodNeedsRestTarget(left_tripod)) {
    active_phase = GaitPhase::RISING;
  }

  if (
    active_phase == GaitPhase::RISING &&
    legs_[sample_leg_index].foot_relative_position.z() + dt * params_.vertical_velocity >=
    params_.foot_z_up)
  {
    active_phase = GaitPhase::UP;
  }

  if (active_phase == GaitPhase::UP && !tripodNeedsRestTarget(left_tripod)) {
    active_phase = GaitPhase::FALLING;
  }

  for (int i = 0; i < kNumLegs; i++) {
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
        moveFootHorizontalToTarget(p, target.x(), target.y(), params_.stop_recenter_velocity, dt);
        break;
      case GaitPhase::UP:
        p.z(params_.foot_z_up);
        moveFootHorizontalToTarget(p, target.x(), target.y(), params_.stop_recenter_velocity, dt);
        break;
      case GaitPhase::FALLING:
        p.z(std::max(params_.foot_z_down, p.z() - params_.vertical_velocity * dt));
        moveFootHorizontalToTarget(p, target.x(), target.y(), params_.stop_recenter_velocity, dt);
        break;
    }
  }

  if (active_phase == GaitPhase::UP && !tripodNeedsRestTarget(left_tripod)) {
    active_phase = GaitPhase::FALLING;
  }

  if (
    active_phase == GaitPhase::FALLING &&
    legs_[sample_leg_index].foot_relative_position.z() <= params_.foot_z_down + params_.velocity_epsilon)
  {
    active_phase = GaitPhase::DOWN;
    for (int i = 0; i < kNumLegs; i++) {
      if (isTripodPhaseRepresentative(left_tripod, i)) {
        legs_[i].foot_relative_position = rest_pose_targets_[i];
      }
    }
  }
}

void TripodGait::beginStopSequence()
{
  cmd_vel_smoothed_ = zeroTwist();
  clearContinuousSwingState();
  stop_state_ = allLegsDown() ? StopState::HOLDING : StopState::LANDING;
}

void TripodGait::beginWalkingSequence(const KDL::Twist & command)
{
  rest_sequence_active_ = false;
  if (stop_state_ != StopState::WALKING && allLegsDown()) {
    const bool start_with_right = chooseStartWithRightTripod(command);
    if (params_.ready_stance_enabled && allLegsAtRestPose() && readyStanceApplies(command)) {
      setReadyStanceForCommand(command, start_with_right);
    }
    gait_cycle_phase_ = start_with_right ? 0.5 : 0.0;
    start_with_right_tripod_ = !start_with_right;
    updatePhaseLabelsFromCycle();
  }
  stop_state_ = StopState::WALKING;
}

void TripodGait::holdRestPose()
{
  left_phase_ = GaitPhase::DOWN;
  right_phase_ = GaitPhase::DOWN;
  clearContinuousSwingState();
  for (int i = 0; i < kNumLegs; i++) {
    legs_[i].foot_relative_position = rest_pose_targets_[i];
  }
}

void TripodGait::updateRestSequence(const double dt)
{
  cmd_vel_smoothed_ = zeroTwist();
  body_pose_current_ = zeroTwist();

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

void TripodGait::updateStopSequence(const double dt)
{
  cmd_vel_smoothed_ = zeroTwist();

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

std::vector<KDL::Vector> TripodGait::computeFootVelocities()
{
  std::vector<KDL::Vector> foot_velocities(kNumLegs);
  double max_vel_ratio = 0.0;
  for (int i = 0; i < kNumLegs; i++) {
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

double TripodGait::computeGaitPhaseRate(
  const std::vector<KDL::Vector> & foot_velocities) const
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

double TripodGait::smoothSwingHeight(const double swing_progress) const
{
  return 0.5 * (1.0 - std::cos(2.0 * M_PI * std::clamp(swing_progress, 0.0, 1.0)));
}

double TripodGait::smoothSwingTravel(const double swing_progress) const
{
  return std::clamp(swing_progress, 0.0, 1.0);
}

void TripodGait::moveLegsContinuous(
  const double dt,
  const std::vector<KDL::Vector> & foot_velocities)
{
  for (int i = 0; i < kNumLegs; i++) {
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
        legs_[i].swing_goal_position = -getRectangleBoundaryGoal(foot_velocities[i]);
        clampFootXY(legs_[i].swing_goal_position);
      }
      const KDL::Vector & goal = legs_[i].swing_goal_position;
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

void TripodGait::updateContinuousGait(
  const double dt,
  const std::vector<KDL::Vector> & foot_velocities)
{
  gait_cycle_phase_ = wrappedPhase(gait_cycle_phase_ + computeGaitPhaseRate(foot_velocities) * dt);
  updatePhaseLabelsFromCycle();
  moveLegsContinuous(dt, foot_velocities);
}

}  // namespace antsy_control
