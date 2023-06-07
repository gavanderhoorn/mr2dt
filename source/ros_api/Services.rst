..
   SPDX-FileCopyrightText: 2023, Yaskawa America, Inc.
   SPDX-FileCopyrightText: 2023, Delft University of Technology

   SPDX-License-Identifier: CC-BY-SA-4.0


########
Services
########

Provided services
=================

read_group_io
-------------

Type: `motoros2_interfaces/srv/ReadGroupIO <https://github.com/yaskawa-global/motoros2_interfaces/blob/d6805d32714df4430f7db3d8ddc736c340ddeba8/srv/ReadGroupIO.srv>`__

Retrieve the state/value of the addressed Group IO element.

Please refer to the documentation embedded in the service definition for more information about addressing and general service behaviour.

read_mregister
--------------

Type: `motoros2_interfaces/srv/ReadMRegister <https://github.com/yaskawa-global/motoros2_interfaces/blob/d6805d32714df4430f7db3d8ddc736c340ddeba8/srv/ReadMRegister.srv>`__

Retrieve the value of the addressed M register.

Please refer to the documentation embedded in the service definition for more information about addressing and general service behaviour.

read_single_io
--------------

Type: `motoros2_interfaces/srv/ReadSingleIO <https://github.com/yaskawa-global/motoros2_interfaces/blob/d6805d32714df4430f7db3d8ddc736c340ddeba8/srv/ReadSingleIO.srv>`__

Retrieve the state/value of the addressed IO element.

Please refer to the documentation embedded in the service definition for more information about addressing and general service behaviour.

reset_error
-----------

Type: `motoros2_interfaces/srv/ResetError <https://github.com/yaskawa-global/motoros2_interfaces/blob/d6805d32714df4430f7db3d8ddc736c340ddeba8/srv/ResetError.srv>`__

Attempt to reset controller errors and alarms.

Inspect the ``result_code`` field to determine the result of the invocation, and check the relevant fields of the ``RobotStatus`` messages to determine overall controller status.

Note: errors and alarms which require physical operator intervention (e-stops, etc) can not be reset by this service.

start_traj_mode
---------------

Type: `motoros2_interfaces/srv/StartTrajMode <https://github.com/yaskawa-global/motoros2_interfaces/blob/d6805d32714df4430f7db3d8ddc736c340ddeba8/srv/StartTrajMode.srv>`__

Attempt to enable servo drives and activate trajectory mode, allowing the action server (``follow_joint_trajectory``, see below) to execute incoming ``FollowJointTrajectory`` action goals.

Note: this service may fail if controller state prevents it from transitioning to trajectory mode.
Inspect the ``result_code`` to determine the cause.
Check the relevant fields of the ``RobotStatus`` messages to determine overall controller status.

The ``reset_error`` service can be used to attempt to reset errors and alarms.

start_point_queue_mode
----------------------

Type: `motoros2_interfaces/srv/StartPointQueueMode <https://github.com/yaskawa-global/motoros2_interfaces/blob/d6805d32714df4430f7db3d8ddc736c340ddeba8/srv/StartPointQueueMode.srv>`__

Attempt to enable servo drives and activate the point-queue motion mode, allowing the ``queue_traj_point`` service to execute incoming ``QueueTrajPoint`` requests.

Note: this service may fail if controller state prevents it from transitioning to trajectory mode.
Inspect the ``result_code`` to determine the cause.
Check the relevant fields of the ``RobotStatus`` messages to determine overall controller status.

The ``reset_error`` service can be used to attempt to reset errors and alarms

stop_traj_mode
--------------

Type: `std_srvs/srv/Trigger <https://github.com/ros2/common_interfaces/blob/37ebe90cbfa91bcdaf69d6ed39c08859c4c3bcd4/std_srvs/srv/Trigger.srv>`__

Attempt to deactivate motion mode.
The servo drives will remain enabled, if they were enabled before this service was called.

This service will fail if called while motion is being executed.

To stop a currently executing FollowJointTrajectory motion, cancel the active goal (either using the action client which submitted it, or after inspecting the list of active goals of the action server and submitting a cancel request for a specific goal id).

queue_traj_point
----------------

Type: `motoros2_interfaces/srv/QueueTrajPoint <https://github.com/yaskawa-global/motoros2_interfaces/blob/d6805d32714df4430f7db3d8ddc736c340ddeba8/srv/QueueTrajPoint.srv>`__

Submit a ``JointTrajectoryPoint`` to be queued for continuous motion.

The ``start_point_queue_mode`` service must have been called prior to attempting this service.

If this service fails, inspect the ``QueueResultEnum`` field in the reply to determine the cause.
The most common type of failure is ``BUSY``.
This is caused when the system is still processing a previously queued point.

write_group_io
--------------

Type: `motoros2_interfaces/srv/WriteGroupIO <https://github.com/yaskawa-global/motoros2_interfaces/blob/d6805d32714df4430f7db3d8ddc736c340ddeba8/srv/WriteGroupIO.srv>`__

Write a value to the addressed Group IO element.

Please refer to the documentation embedded in the service definition for more information about legal values, addressing and general service behaviour.

write_mregister
---------------

Type: `motoros2_interfaces/srv/WriteMRegister <https://github.com/yaskawa-global/motoros2_interfaces/blob/d6805d32714df4430f7db3d8ddc736c340ddeba8/srv/WriteMRegister.srv>`__

Write a value to the addressed M register.

Please refer to the documentation embedded in the service definition for more information about legal values, addressing and general service behaviour.

write_single_io
---------------

Type: `motoros2_interfaces/srv/WriteSingleIO <https://github.com/yaskawa-global/motoros2_interfaces/blob/d6805d32714df4430f7db3d8ddc736c340ddeba8/srv/WriteSingleIO.srv>`__

Write a value to the addressed IO element.

Please refer to the documentation embedded in the service definition for more information about legal values, addressing and general service behaviour.

Services called
===============

None.
