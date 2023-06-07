..
   SPDX-FileCopyrightText: 2023, Yaskawa America, Inc.
   SPDX-FileCopyrightText: 2023, Delft University of Technology

   SPDX-License-Identifier: CC-BY-SA-4.0


######
Topics
######

Subscribed topics
=================

None.

Published topics
================

joint_states
------------

Type: `sensor_msgs/msg/JointState <https://github.com/ros2/common_interfaces/blob/37ebe90cbfa91bcdaf69d6ed39c08859c4c3bcd4/sensor_msgs/msg/JointState.msg>`__

Joint state for all joints in all groups of the connected robot:

-  joint position (rad or metre)
-  joint velocity (rad/sec or metres/sec)
-  joint effort: torque for revolute joints (Nm), force for prismatic joints (N)

ctrl_groups/…/joint_states
--------------------------

Joint states for the joints in a specific motion group (fi: ``r1``, ``b1``, ``s1``, etc).
One topic per configured motion group.
``JointState`` messages published on these topics will only contain information on the joints in the motion group.

This topic carries the same message type as the global ``joint_states`` topic.

robot_status
------------

Type: `industrial_msgs/msg/RobotStatus <https://github.com/ros-industrial/industrial_core/blob/d547cdcfdaf3bc0d46325215b8219b0a190c8e6c/industrial_msgs/msg/RobotStatus.msg>`__

Aggregate controller/robot status (ie: drives enabled, motion possible, active error, etc).

Use this topic (in addition to the ``result_code``\ s) to determine whether there are any error conditions preventing ``start_traj_mode`` from activating the servos and subsequently enabling trajectory mode.

tf
--

Type: `tf2_msgs/msg/TFMessage <https://github.com/ros2/geometry2/blob/51a7f24191198eb9fc8124d36aba5bb2f7ad84f3/tf2_msgs/msg/TFMessage.msg>`__

Standard ROS topic onto which TF transforms are broadcast.

Note: this topic is only namespaced if a namespace is configured *and* ``namespace_tf`` is set to ``true`` in the configuration file.
