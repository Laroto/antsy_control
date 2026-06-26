# antsy_control

Control package for ANTSY. The main runtime node is `follow_velocity_rectangle`, which turns `cmd_vel` into tripod gait leg commands and publishes `/actuators`. This repo also contains a simple trajectory player, `walk_forward`, and the real-robot launch file that composes description, control, and the Hiwonder writer.

## Nodes

### `follow_velocity_rectangle`

Launch it with:

```bash
ros2 launch antsy_control follow_velocity_rectangle.launch.xml
```

Main topics and services:

- subscribes: `cmd_vel`, `cmd_vel_unstamped`, `body_pose_mode`, `robot_description`
- optionally subscribes: `heading_hold.odom_topic`, `leg_odometry.joint_states_topic`
- publishes: `actuators`, `cmd_vel_adj`, `relative_foot_positions`, `gait_phase_left`, `gait_phase_right`, `leg_odom`
- services: `go_to_rest_pose`, `/control/reset`

Parameters:

| Parameter | Default | Meaning |
| --- | --- | --- |
| `control.period` | `0.02` | Main controller update period in seconds. |
| `cmd_vel_timeout` | `0.3` | If no fresh command arrives within this time, the controller holds stance. |
| `control.max_dt` | `0.05` | Upper bound used when integrating controller time steps after scheduling jitter. |
| `gait.max_velocity_ratio` | `0.50` | Global scaling cap applied when requested stance foot speed exceeds configured limits. |
| `gait.step_limit_x` | `0.045` | Maximum foot travel in X, relative to each leg neutral point. |
| `gait.step_limit_y` | `0.035` | Maximum foot travel in Y, relative to each leg neutral point. |
| `gait.foot_z_down` | `-0.100` | Cartesian Z used when a foot is planted. |
| `gait.foot_z_up` | `-0.070` | Cartesian Z used at the top of swing. |
| `gait.foot_z_sync` | `-0.090` | Z threshold where falling legs switch from swing tracking to stance tracking. |
| `gait.stance_velocity_x` | `0.67` | Maximum nominal stance-foot X speed before the command is scaled down. |
| `gait.stance_velocity_y` | `0.34` | Maximum nominal stance-foot Y speed before the command is scaled down. |
| `gait.vertical_velocity` | `0.2266666667` | Foot lift/drop speed in meters per second in Cartesian leg space. |
| `gait.swing_xy_velocity` | `0.55` | Horizontal speed used to move swing legs toward their swing target. |
| `gait.idle_return_velocity` | `0.25` | Idle recovery speed used when returning legs without active walking. |
| `gait.stop_recenter_tolerance` | `0.020` | XY error tolerance before stopped legs are replanted toward center/rest targets. |
| `gait.stop_recenter_velocity` | `0.20` | XY replant speed used during stop/rest recentering. |
| `gait.ready_stance_enabled` | `true` | If enabled, the controller pre-positions feet into a better gait start stance. |
| `gait.ready_stance_linear_x` | `1.0` | Stored ready-stance tuning value for forward starts. |
| `gait.ready_stance_linear_y` | `0.0` | Stored ready-stance tuning value for lateral starts. |
| `gait.ready_stance_angular_z` | `0.0` | Stored ready-stance tuning value for yaw starts. |
| `gait.velocity_epsilon` | `1e-5` | Small threshold used to decide whether a command or foot velocity is effectively zero. |
| `command_filter.max_linear_acceleration` | `0.8` | Linear slew-rate limit applied to walking commands. |
| `command_filter.max_angular_acceleration` | `2.0` | Angular slew-rate limit applied to walking commands. |
| `body_pose.max_x` | `0.030` | Maximum body-pose mode X translation command. |
| `body_pose.max_y` | `0.025` | Maximum body-pose mode Y translation command. |
| `body_pose.max_z` | `0.020` | Maximum body-pose mode Z translation command. |
| `body_pose.max_roll` | `0.18` | Maximum body-pose mode roll command in radians. |
| `body_pose.max_pitch` | `0.18` | Maximum body-pose mode pitch command in radians. |
| `body_pose.max_yaw` | `0.22` | Maximum body-pose mode yaw command in radians. |
| `body_pose.linear_rate` | `0.10` | Rate limit for body-pose linear offsets. |
| `body_pose.angular_rate` | `0.70` | Rate limit for body-pose angular offsets. |
| `ik.position_weight` | `1.0` | Position task weight passed to the KDL IK solver. |
| `ik.orientation_weight` | `0.0` | Orientation task weight passed to the KDL IK solver. |
| `ik.seed_joint_angles_left` | `[0.0, 0.6, 1.8]` | Initial IK seed used on left-side legs. |
| `ik.seed_joint_angles_right` | `[0.0, -0.6, -1.8]` | Initial IK seed used on right-side legs. |
| `heading_hold.enabled` | `true` | Enables heading correction during translational walking. |
| `heading_hold.kp` | `1.8` | Proportional heading correction gain. |
| `heading_hold.kd` | `0.30` | Derivative heading correction gain using yaw rate. |
| `heading_hold.max_angular_velocity` | `0.40` | Clamp on heading-hold yaw correction. |
| `heading_hold.min_linear_velocity` | `0.02` | Minimum translational speed before heading hold activates. |
| `heading_hold.angular_deadband` | `0.02` | If commanded yaw exceeds this magnitude, heading hold is bypassed. |
| `heading_hold.odom_topic` | `"leg_odom"` | Odometry topic used for heading hold. Leave it on `leg_odom` for real use. |
| `leg_odometry.enabled` | `true` | Enables publication of `leg_odom`. |
| `leg_odometry.reset_pose_on_control_reset` | `true` | If true, `/control/reset` also zeros the integrated leg odometry pose. |
| `leg_odometry.topic` | `"leg_odom"` | Output odometry topic name. |
| `leg_odometry.frame_id` | `"odom"` | Frame id used in published `nav_msgs/Odometry`. |
| `leg_odometry.child_frame_id` | `"base_link"` | Child frame id used in published `nav_msgs/Odometry`. |
| `leg_odometry.joint_states_topic` | `"joint_states"` | Joint-state topic used for measured-foot odometry when available. |
| `leg_odometry.use_joint_states` | `true` | If true, use measured joint states for FK-based leg odometry when possible. |
| `leg_odometry.min_support_legs` | `3` | Minimum number of stance legs required for an odometry update. |
| `leg_odometry.max_fit_residual` | `0.03` | Maximum rigid-fit residual accepted for a leg-odometry update. |
| `leg_odometry.max_linear_delta` | `0.03` | Maximum single-tick XY body displacement accepted before the update is rejected as an outlier. |
| `leg_odometry.max_angular_delta` | `0.25` | Maximum single-tick yaw change accepted before the update is rejected as an outlier. |
| `leg_odometry.translation_scale` | `1.0` | Scalar applied to each accepted XY odometry increment. Use this to calibrate systematic stance-slip or model-scale bias. |
| `leg_odometry.propagate_on_invalid_update` | `false` | If true, dead-reckon short invalid windows from the last accepted leg-odometry twist instead of freezing the estimate. |
| `leg_odometry.max_prediction_time` | `0.08` | Maximum total dead-reckoning time allowed while `propagate_on_invalid_update` is active. |

### `walk_forward`

Run it with:

```bash
ros2 run antsy_control walk_forward --ros-args -p trajectory_filepath:=/abs/path/to/file.csv
```

Topics:

- subscribes: `robot_description`
- publishes: `actuators`

Parameters:

| Parameter | Default | Meaning |
| --- | --- | --- |
| `trajectory_filepath` | `""` | CSV file containing the leg trajectory used by the demo gait player. The node throws if it is empty or unreadable. |

### `walk_sideways_while_rotating`

This is only a helper publisher script for quick testing. It does not declare ROS parameters.

## Launch files

### `follow_velocity_rectangle.launch.xml`

Arguments:

| Argument | Default | Meaning |
| --- | --- | --- |
| `use_sim_time` | `true` | Passed to `follow_velocity_rectangle` as the standard ROS simulation-clock parameter. |

### `real_robot.launch.py`

This launch file starts the description, the controller, and `hiwonder_ros2/write_only`.

Arguments:

| Argument | Default | Meaning |
| --- | --- | --- |
| `use_sim_time` | `false` | Passed into description and control. Keep `false` on hardware. |
| `hiwonder_device` | `"/dev/ttyUSB0"` | Serial device passed to the Hiwonder writer node. |
| `hiwonder_baud_rate` | `"9600"` | Serial baud rate passed to the Hiwonder writer node. |
| `hiwonder_write_rate` | `"4"` | Passed into the Hiwonder writer as `motor_read_rate`, which sets its write timer period. |

## Body-pose mode

In body-pose mode, `cmd_vel` is interpreted as a body pose command instead of a walking command:

- `linear.x`, `linear.y`, `linear.z`: body XYZ offsets
- `angular.x`, `angular.y`, `angular.z`: roll, pitch, yaw

Useful commands:

```bash
ros2 topic pub --once /body_pose_mode std_msgs/msg/Bool "{data: true}"
ros2 topic pub --once /body_pose_mode std_msgs/msg/Bool "{data: false}"
ros2 service call /go_to_rest_pose std_srvs/srv/Trigger "{}"
ros2 service call /control/reset std_srvs/srv/Trigger "{}"
```
