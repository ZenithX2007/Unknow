# Sweeper integration

This package is the ROS 2 Humble boundary between the Gen0 simulator and the
navigation / cleaning stack. It owns the current odometry, command adaptation,
SLAM, Nav2, and exploration launch flow for the `san_roundabout` demo.

## Topic contract

- Input pose: `/gen0_model/links/poses` (`geometry_msgs/PoseArray`)
- Output odometry: `/odom` (`nav_msgs/Odometry`)
- Navigation and Gen0 command: `/cmd_vel` (`geometry_msgs/Twist`)
- Gazebo bridge: ROS `/cmd_vel` -> Gazebo Ackermann `/cmd_vel`
- TF: `odom -> base_footprint -> base_link`

The initial `pose_index` is 15 because the original Gen0 ground-truth node used
that index. Verify the reported PoseArray size and vehicle motion before using
the odometry for navigation.
