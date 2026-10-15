# CTM2F110 ROS 2 driver

This package is the ROS 2 counterpart of the vendor ROS 1 `gripper_listener` node. It keeps the vendor SDK and its calibrated conversion (`servo_position = 1700 - 700 * joint_position`) and exposes:

- `control_msgs/action/FollowJointTrajectory` at `~/follow_joint_trajectory`;
- a six-joint `sensor_msgs/msg/JointState` stream at `/joint_states`;
- `dry_run=true` by default for safe offline testing.

The `readback` parameter is enabled by default for hardware. It uses the vendor `FeedBack` plus
`ReadPos(-1)` calls to publish measured actuator position. Set `readback:=false` only when the
device does not support present-position reads; the node then publishes the last commanded position.

The action accepts one active joint (`ctm2f110_finger_joint` by default), validates every point
against the configured position limits, respects `time_from_start`, and supports cancellation.
The vendor protocol wrapper reports failed register writes as an aborted action. In dry-run mode
the state is commanded position; in hardware mode with readback enabled it is the average of actuator
IDs 1 and 2 after calibrated conversion. A failed read retains the prior value and emits one warning.

For hardware, first verify the serial device and then launch:

```bash
ros2 launch changingtek_gripper_driver ctm2f110_driver.launch.py \
  serial_port:=/dev/ttyUSB0 dry_run:=false
```

Do not run this node together with the Gazebo controller or another node publishing `/joint_states`. The MoveIt controller configuration should target the namespaced action `ctm2f110_hand_controller/follow_joint_trajectory` when using the hardware launch.
