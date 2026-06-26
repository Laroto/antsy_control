#include "antsy_control/leg_odometry.hpp"

#include <cmath>
#include <stdexcept>

namespace antsy_control
{

namespace
{
double normalizeAngle(double angle)
{
  while (angle > M_PI) {
    angle -= 2.0 * M_PI;
  }
  while (angle < -M_PI) {
    angle += 2.0 * M_PI;
  }
  return angle;
}
}  // namespace

LegOdometryEstimator::LegOdometryEstimator(const LegOdometryParameters & params)
: params_(params)
{
  if (
    params_.min_support_legs < 1 ||
    params_.max_fit_residual < 0.0 ||
    params_.max_linear_delta <= 0.0 ||
    params_.max_angular_delta <= 0.0 ||
    params_.translation_scale <= 0.0 ||
    params_.max_prediction_time < 0.0)
  {
    throw std::runtime_error("Invalid leg odometry parameters.");
  }
}

void LegOdometryEstimator::reset(const bool reset_pose)
{
  previous_foot_positions_.clear();
  previous_stance_legs_.clear();
  has_previous_sample_ = false;
  state_.vx = 0.0;
  state_.vy = 0.0;
  state_.wz = 0.0;
  state_.support_legs = 0;
  state_.fit_residual = 0.0;
  state_.valid_update = false;
  has_twist_estimate_ = false;
  predicted_duration_sec_ = 0.0;

  if (reset_pose) {
    state_.x = 0.0;
    state_.y = 0.0;
    state_.yaw = 0.0;
  }
}

const LegOdometryState & LegOdometryEstimator::state() const
{
  return state_;
}

LegOdometryState LegOdometryEstimator::update(
  const std::vector<KDL::Vector> & foot_positions,
  const std::vector<bool> & stance_legs,
  const bool freeze,
  const double dt)
{
  if (foot_positions.size() != stance_legs.size()) {
    throw std::runtime_error("Leg odometry foot position and stance arrays have different sizes.");
  }

  const double previous_vx = state_.vx;
  const double previous_vy = state_.vy;
  const double previous_wz = state_.wz;
  state_.valid_update = false;
  state_.vx = 0.0;
  state_.vy = 0.0;
  state_.wz = 0.0;

  if (freeze || !has_previous_sample_ || dt <= 0.0 || !std::isfinite(dt)) {
    predicted_duration_sec_ = 0.0;
    rememberSample(foot_positions, stance_legs);
    return state_;
  }

  const Transform2D delta = estimateBodyDelta(foot_positions, stance_legs);
  state_.support_legs = delta.support_legs;
  state_.fit_residual = delta.residual;

  const double linear_delta = std::hypot(delta.x, delta.y);
  if (
    delta.valid &&
    delta.residual <= params_.max_fit_residual &&
    linear_delta <= params_.max_linear_delta &&
    std::abs(delta.yaw) <= params_.max_angular_delta)
  {
    integrateDelta(delta, dt, true);
    has_twist_estimate_ = true;
    predicted_duration_sec_ = 0.0;
  } else if (
    params_.propagate_on_invalid_update &&
    has_twist_estimate_ &&
    predicted_duration_sec_ + dt <= params_.max_prediction_time)
  {
    Transform2D predicted_delta;
    predicted_delta.x = previous_vx * dt;
    predicted_delta.y = previous_vy * dt;
    predicted_delta.yaw = previous_wz * dt;
    integrateDelta(predicted_delta, dt, false);
    predicted_duration_sec_ += dt;
  }

  rememberSample(foot_positions, stance_legs);
  return state_;
}

LegOdometryEstimator::Transform2D LegOdometryEstimator::estimateBodyDelta(
  const std::vector<KDL::Vector> & foot_positions,
  const std::vector<bool> & stance_legs) const
{
  Transform2D delta;
  if (!has_previous_sample_) {
    return delta;
  }

  double previous_cx = 0.0;
  double previous_cy = 0.0;
  double current_cx = 0.0;
  double current_cy = 0.0;
  for (size_t i = 0; i < foot_positions.size(); i++) {
    if (!stance_legs[i] || !previous_stance_legs_[i]) {
      continue;
    }
    previous_cx += previous_foot_positions_[i].x();
    previous_cy += previous_foot_positions_[i].y();
    current_cx += foot_positions[i].x();
    current_cy += foot_positions[i].y();
    delta.support_legs++;
  }

  if (delta.support_legs < params_.min_support_legs) {
    return delta;
  }

  previous_cx /= delta.support_legs;
  previous_cy /= delta.support_legs;
  current_cx /= delta.support_legs;
  current_cy /= delta.support_legs;

  double cross = 0.0;
  double dot = 0.0;
  for (size_t i = 0; i < foot_positions.size(); i++) {
    if (!stance_legs[i] || !previous_stance_legs_[i]) {
      continue;
    }
    const double cx = foot_positions[i].x() - current_cx;
    const double cy = foot_positions[i].y() - current_cy;
    const double px = previous_foot_positions_[i].x() - previous_cx;
    const double py = previous_foot_positions_[i].y() - previous_cy;
    cross += cx * py - cy * px;
    dot += cx * px + cy * py;
  }

  delta.yaw = std::atan2(cross, dot);
  const double cos_delta = std::cos(delta.yaw);
  const double sin_delta = std::sin(delta.yaw);
  delta.x = previous_cx - (cos_delta * current_cx - sin_delta * current_cy);
  delta.y = previous_cy - (sin_delta * current_cx + cos_delta * current_cy);

  double residual_sum = 0.0;
  for (size_t i = 0; i < foot_positions.size(); i++) {
    if (!stance_legs[i] || !previous_stance_legs_[i]) {
      continue;
    }
    const double predicted_x =
      cos_delta * foot_positions[i].x() - sin_delta * foot_positions[i].y() + delta.x;
    const double predicted_y =
      sin_delta * foot_positions[i].x() + cos_delta * foot_positions[i].y() + delta.y;
    const double rx = previous_foot_positions_[i].x() - predicted_x;
    const double ry = previous_foot_positions_[i].y() - predicted_y;
    residual_sum += std::hypot(rx, ry);
  }

  delta.residual = residual_sum / delta.support_legs;
  delta.valid = true;
  return delta;
}

void LegOdometryEstimator::integrateDelta(
  const Transform2D & delta,
  const double dt,
  const bool valid_update)
{
  const double scaled_x = delta.x * params_.translation_scale;
  const double scaled_y = delta.y * params_.translation_scale;
  const double cos_yaw = std::cos(state_.yaw);
  const double sin_yaw = std::sin(state_.yaw);
  state_.x += cos_yaw * scaled_x - sin_yaw * scaled_y;
  state_.y += sin_yaw * scaled_x + cos_yaw * scaled_y;
  state_.yaw = normalizeAngle(state_.yaw + delta.yaw);
  state_.vx = scaled_x / dt;
  state_.vy = scaled_y / dt;
  state_.wz = delta.yaw / dt;
  state_.valid_update = valid_update;
}

void LegOdometryEstimator::rememberSample(
  const std::vector<KDL::Vector> & foot_positions,
  const std::vector<bool> & stance_legs)
{
  previous_foot_positions_ = foot_positions;
  previous_stance_legs_ = stance_legs;
  has_previous_sample_ = true;
}

}  // namespace antsy_control
