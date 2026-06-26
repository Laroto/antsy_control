#ifndef ANTSY_CONTROL__TRIPOD_GAIT_HPP_
#define ANTSY_CONTROL__TRIPOD_GAIT_HPP_

#include <cstdint>
#include <vector>

#include "kdl/frames.hpp"
#include "kdl/jntarray.hpp"

namespace antsy_control
{

constexpr int kNumLegs = 6;
constexpr int kNumJointsPerLeg = 3;

enum class GaitPhase : uint8_t
{
  DOWN = 0,
  RISING = 1,
  UP = 2,
  FALLING = 3,
};

struct Leg
{
  KDL::Vector foot_center_position;
  KDL::Vector foot_relative_position;
  KDL::Vector swing_start_position;
  bool in_continuous_swing = false;
  KDL::JntArray joint_angles;
};

struct TripodGaitParameters
{
  double max_velocity_ratio = 0.50;
  double step_limit_x = 0.045;
  double step_limit_y = 0.035;
  double foot_z_down = -0.100;
  double foot_z_up = -0.070;
  double foot_z_sync = -0.090;
  double stance_velocity_x = 0.67;
  double stance_velocity_y = 0.34;
  double vertical_velocity = 0.2266666667;
  double swing_xy_velocity = 0.55;
  double idle_return_velocity = 0.25;
  double stop_recenter_tolerance = 0.020;
  double stop_recenter_velocity = 0.20;
  bool ready_stance_enabled = true;
  double ready_stance_linear_x = 1.0;
  double ready_stance_linear_y = 0.0;
  double ready_stance_angular_z = 0.0;
  double max_linear_acceleration = 0.8;
  double max_angular_acceleration = 2.0;
  double body_pose_max_x = 0.030;
  double body_pose_max_y = 0.025;
  double body_pose_max_z = 0.020;
  double body_pose_max_roll = 0.18;
  double body_pose_max_pitch = 0.18;
  double body_pose_max_yaw = 0.22;
  double body_pose_linear_rate = 0.10;
  double body_pose_angular_rate = 0.70;
  double velocity_epsilon = 1e-5;
};

KDL::Twist zeroTwist();

class TripodGait
{
public:
  explicit TripodGait(const TripodGaitParameters & params);

  void reset();
  void setBodyPoseModeEnabled(bool enabled);
  bool bodyPoseModeEnabled() const;
  void beginRestSequence();
  void update(double dt, const KDL::Twist & command, bool command_timed_out);

  const std::vector<Leg> & legs() const;
  std::vector<Leg> & mutableLegs();
  const KDL::Twist & smoothedCommand() const;
  GaitPhase phaseForLeg(int leg_index) const;
  bool supportForLeg(int leg_index) const;
  GaitPhase leftPhase() const;
  GaitPhase rightPhase() const;
  bool allLegsDown() const;

private:
  enum class StopState
  {
    WALKING = 0,
    LANDING = 1,
    REPLANT_LEFT = 2,
    REPLANT_RIGHT = 3,
    HOLDING = 4,
  };

  enum class RestState
  {
    LANDING = 0,
    REPLANT_LEFT = 1,
    REPLANT_RIGHT = 2,
    HOLDING = 3,
  };

  void validateParameters() const;
  void initializeLegs();
  void clearContinuousSwingState();
  bool commandActive(const KDL::Twist & command) const;
  bool footMotionActive(const std::vector<KDL::Vector> & foot_velocities) const;
  bool isTripodPhaseRepresentative(bool left_tripod, int leg_index) const;
  void clampFootXY(KDL::Vector & p) const;
  double moveTowards(double current, double target, double max_delta) const;
  double clampSymmetric(double value, double limit) const;
  double wrappedPhase(double phase) const;
  double tripodPhase(bool left_tripod) const;
  GaitPhase phaseLabelFromTripodPhase(double phase) const;
  void updatePhaseLabelsFromCycle();
  double durationToLimit(double position, double velocity, double limit) const;
  KDL::Vector getRectangleBoundaryGoal(const KDL::Vector & direction) const;
  void moveSwingFootHorizontal(KDL::Vector & p, const KDL::Vector & foot_velocity, double dt) const;
  void moveFootHorizontalToTarget(
    KDL::Vector & p, double goal_x, double goal_y, double max_speed, double dt) const;

  double tripodMaxOffset(bool left_tripod) const;
  bool tripodNeedsRecentering(bool left_tripod) const;
  StopState chooseReplantState() const;
  double tripodRestMaxError(bool left_tripod) const;
  bool tripodNeedsRestTarget(bool left_tripod) const;
  bool allLegsAtRestPose() const;
  RestState chooseRestState() const;

  KDL::Vector footVelocityForCommand(int leg_index, const KDL::Twist & command) const;
  std::vector<KDL::Vector> computeFootVelocitiesForCommand(const KDL::Twist & command) const;
  double readyStanceError(
    const std::vector<KDL::Vector> & foot_velocities, bool start_with_right) const;
  bool chooseStartWithRightTripod(const KDL::Twist & command) const;
  void setReadyStanceForCommand(const KDL::Twist & command, bool start_with_right);
  KDL::Twist limitCommandRate(const KDL::Twist & current, const KDL::Twist & target, double dt) const;
  KDL::Twist bodyPoseTargetFromCommand(const KDL::Twist & command, bool command_timed_out) const;
  KDL::Twist limitBodyPoseRate(const KDL::Twist & current, const KDL::Twist & target, double dt) const;
  void updateBodyPoseMode(double dt, const KDL::Twist & command, bool command_timed_out);
  void holdCurrentPose();
  void landSwingFeetInPlace(double dt);
  void replantTripod(bool left_tripod, double dt);
  void replantTripodToRestTargets(bool left_tripod, double dt);
  void beginStopSequence();
  void beginWalkingSequence(const KDL::Twist & command);
  void holdRestPose();
  void updateRestSequence(double dt);
  void updateStopSequence(double dt);
  std::vector<KDL::Vector> computeFootVelocities();
  double computeGaitPhaseRate(const std::vector<KDL::Vector> & foot_velocities) const;
  double smoothSwingHeight(double swing_progress) const;
  double smoothSwingTravel(double swing_progress) const;
  void moveLegsContinuous(double dt, const std::vector<KDL::Vector> & foot_velocities);
  void updateContinuousGait(double dt, const std::vector<KDL::Vector> & foot_velocities);

  TripodGaitParameters params_;
  std::vector<Leg> legs_;
  std::vector<KDL::Vector> rest_pose_targets_;
  KDL::Twist cmd_vel_smoothed_;
  KDL::Twist body_pose_current_;
  bool body_pose_mode_enabled_ = false;
  StopState stop_state_ = StopState::HOLDING;
  bool rest_sequence_active_ = false;
  RestState rest_state_ = RestState::HOLDING;
  double gait_cycle_phase_ = 0.0;
  bool start_with_right_tripod_ = true;
  GaitPhase left_phase_ = GaitPhase::DOWN;
  GaitPhase right_phase_ = GaitPhase::DOWN;

  static constexpr double swing_phase_fraction_ = 0.5;
};

}  // namespace antsy_control

#endif  // ANTSY_CONTROL__TRIPOD_GAIT_HPP_
