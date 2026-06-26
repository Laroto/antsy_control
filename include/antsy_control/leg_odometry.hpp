#ifndef ANTSY_CONTROL__LEG_ODOMETRY_HPP_
#define ANTSY_CONTROL__LEG_ODOMETRY_HPP_

#include <vector>

#include "kdl/frames.hpp"

namespace antsy_control
{

struct LegOdometryParameters
{
  int min_support_legs = 3;
  double max_fit_residual = 0.03;
  double max_linear_delta = 0.03;
  double max_angular_delta = 0.25;
  double translation_scale = 1.0;
  bool propagate_on_invalid_update = false;
  double max_prediction_time = 0.08;
};

struct LegOdometryState
{
  double x = 0.0;
  double y = 0.0;
  double yaw = 0.0;
  double vx = 0.0;
  double vy = 0.0;
  double wz = 0.0;
  int support_legs = 0;
  double fit_residual = 0.0;
  bool valid_update = false;
};

class LegOdometryEstimator
{
public:
  explicit LegOdometryEstimator(const LegOdometryParameters & params = LegOdometryParameters{});

  void reset(bool reset_pose);
  const LegOdometryState & state() const;
  LegOdometryState update(
    const std::vector<KDL::Vector> & foot_positions,
    const std::vector<bool> & stance_legs,
    bool freeze,
    double dt);

private:
  struct Transform2D
  {
    double x = 0.0;
    double y = 0.0;
    double yaw = 0.0;
    double residual = 0.0;
    int support_legs = 0;
    bool valid = false;
  };

  Transform2D estimateBodyDelta(
    const std::vector<KDL::Vector> & foot_positions,
    const std::vector<bool> & stance_legs) const;
  void integrateDelta(const Transform2D & delta, double dt, bool valid_update);
  void rememberSample(
    const std::vector<KDL::Vector> & foot_positions,
    const std::vector<bool> & stance_legs);

  LegOdometryParameters params_;
  LegOdometryState state_;
  std::vector<KDL::Vector> previous_foot_positions_;
  std::vector<bool> previous_stance_legs_;
  bool has_previous_sample_ = false;
  bool has_twist_estimate_ = false;
  double predicted_duration_sec_ = 0.0;
};

}  // namespace antsy_control

#endif  // ANTSY_CONTROL__LEG_ODOMETRY_HPP_
