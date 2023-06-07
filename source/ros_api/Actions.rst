..
   SPDX-FileCopyrightText: 2023, Yaskawa America, Inc.
   SPDX-FileCopyrightText: 2023, Delft University of Technology

   SPDX-License-Identifier: CC-BY-SA-4.0

#######
Actions
#######

Action servers
==============

follow_joint_trajectory
-----------------------

Type: `control_msgs/action/FollowJointTrajectory <https://github.com/ros-controls/control_msgs/blob/a555c37f1a3536bb452ea555c58fdd9344d87614/control_msgs/action/FollowJointTrajectory.action>`__

Execute the trajectory submitted as part of the goal, under the conditions specified by the goal (only the ``goal_time_tolerance`` and ``goal_tolerance`` fields are supported by MotoROS2 in the current implementation).

MotoROS2 attempts to execute the motion encoded by the `JointTrajectory <https://github.com/ros2/common_interfaces/blob/37ebe90cbfa91bcdaf69d6ed39c08859c4c3bcd4/trajectory_msgs/msg/JointTrajectory.msg>`__ as faithfully as possible.
Due to requirements on the dynamics, accelerations specified are recalculated by MotoROS2 based on segment duration and velocities in each individual ``JointTrajectoryPoint``.

Note: MotoROS2 has extended the possible set of values returned in the ``error_code`` field of the final action result.
Returned error values are always of the form ``-ECCCCC``, where ``E`` is `the ROS defined error code <https://github.com/ros-controls/control_msgs/blob/a555c37f1a3536bb452ea555c58fdd9344d87614/control_msgs/action/FollowJointTrajectory.action#L35-L39>`__ and ``CCCCC`` is `a MotoROS2 error code <https://github.com/yaskawa-global/motoros2_interfaces/blob/d6805d32714df4430f7db3d8ddc736c340ddeba8/msg/MotionReadyEnum.msg>`__.

Example: ``error_code: -100101`` decodes into ``-1`` and ``101``, which would indicate the goal is invalid because there is an active alarm.

Actions called
==============

None.
