
##########################
Known issues & limitations
##########################

MotoROS2 is still under development.
As such, there are currently a number of known issues and limitations users should be aware of and should take into account.

This section documents these and provides work-arounds where possible.

Only FastDDS is supported
=========================

**Description**: the current implementation of MotoROS2 can only communicate with ROS 2 applications which use eProsima FastDDS as their RMW layer.
None of the other RMWs are supported, including Cyclone DDS, which is the default RMW of ROS 2 Galactic.

Symptoms of this incompatibility are seemingly functional ROS 2 network connections, where topics are succesfully published and subscribed to, but service invocations and action goal submissions appear to *hang*.

**Note**: ROS 2 Foxy, ROS 2 Humble and ROS 2 Rolling all use FastDDS by default.
If you haven’t changed your default RMW, you should not need to change anything for MotoROS2.

**Work-around**: unfortunately, this limitation is caused by a middleware-layer incompatibility with respect to how service requests are (de)serialised by the respective RMWs (`ros2/rmw_cyclonedds#184 <https://github.com/ros2/rmw_cyclonedds/issues/184>`__), and without significant changes to the way MotoROS2 operates, has no known work-around.

Users with ROS 2 Galactic installed could install the ``ros-galactic-rmw-fastrtps-cpp`` package and run ``export RMW_IMPLEMENTATION=rmw_fastrtps_cpp`` before using any ROS 2 functionality on their host systems.
Note that the ``RMW_IMPLEMENTATION`` environment variable must be ``export``\ ed to all shells which are used to run ROS 2 applications.

Please note that the ``ros-galactic-rmw-fastrtps-cpp`` package must be install *prior* to building the client application workspace.
If the workspace has already been built, it must be rebuilt after installing the package.

Refer to `Working with multiple ROS 2 middleware implementations <https://docs.ros.org/en/humble/How-To-Guides/Working-with-multiple-RMW-implementations.html>`__ in the ROS 2 documentation for more information about configuring different RMWs with ROS 2.

Maximum length of trajectories
==============================

**Description**: due to controller resource constraints and implementation details of micro-ROS, MotoROS2 imposes an upper limit on the number of ``JointTrajectoryPoint``\ s in ``JointTrajectory``\ s submitted as part of ``control_msgs/FollowJointTrajectory`` action goals.

This maximum number of points in a single trajectory is currently **200**.

This number was derived from testing on a two-robot system (12 axes total).
On larger systems with more control groups, it is possible 200 points may exceed the memory threshold for transmission.

Unfortunately, due to a known issue with micro-ROS (`micro-ROS/micro-ROS-Agent#143 <https://github.com/micro-ROS/micro-ROS-Agent/issues/143>`__), MotoROS2 currently cannot check whether incoming trajectories are too long, nor can MotoROS2 notify the action client in case trajectories in those cases.

Please make sure to check trajectory length *before* submitting goals, as client applications are currently responsible for making sure trajectories do not go over this limit.

**Note**: this is strictly a limit on the *number of trajectory points*, not on the total time duration of a trajectory.

**Work-around**: client applications could split long trajectories into smaller sections, each no longer than the maximum of ``200`` trajectory points.
While motion continuity will not be maintained between trajectories, this approach would allow for longer (as in: longer in time) motions to be commanded by a ROS 2 client.
Whether this would be an acceptable work-around depends on whether the application and the motions it uses support natural stopping points or dwell times.

If the memory threshold is exceeded due to a large trajectory, then the robot will not be notified of the commanded trajectory.
Clients should utilize an appropriate timeout to detect whether the robot responds to a commanded trajectory.

No support for asynchronous motion
==================================

**Description**: MotoROS2 currently only supports synchronous motion across all motion groups configured on the Yaskawa controller.
Controllers with multiple motion groups are supported, but motion groups cannot execute motions independently from each other.

**Work-around**: none at this time.
However, goals which move only a subset of groups (and/or joints) can be created: retrieve the current state of all groups and update target poses for the groups (and/or joints) which should move only.
By keeping the state of all other groups (and/or joints) static, those groups (and/or joints) will not move.

This is not true asynchronous motion, but could allow for some use-cases to still be implemented with the current versions of MotoROS2.

No support for partial goals
============================

**Description**: goals submitted to MotoROS2 *must* always include information for all groups and all joints configured on the controller, even if those groups or joints are not supposed to move.
So called *partial goals* are currently not supported and will be rejected by MotoROS2.

**Work-around**: please refer to the work-around described in `No support for asynchronous motion <#no-support-for-asynchronous-motion>`__.

Upper limit to publishing frequency
===================================

**Description**: the frequency for publishing topics is configurable in the ``motoros2_config.yaml`` configuration file.
Initial testing has revealed that the MotoROS2 is limited to about 100 Hz.

**Work-around**: none at this time.
The issue is being investigated.

Incorrect transform tree origin with multi-robot setups
=======================================================

**Description**: when configured to broadcast TF frames for controllers with multiple robots, the origins of the transform trees for the different robot groups will be coincident, unless the group combination has been calibrated.
For example, an ``R1+R2`` configuration, without calibration, will have the origins of the transform trees for ``R1`` and ``R2`` in the same place, making their TF trees overlap.

**Solution**: perform the calibration for the robot group(s).
Refer to the relevant Yaskawa Motoman documentation for more information on how to perform a robot-to-robot calibration.

After robot-calibration, the transform between the shared ``world`` frame and each robot’s ``base`` frame will be known, and MotoROS2 will include it in the transforms it broadcasts.

Not compatible with Simple Connect
==================================

**Description**: *Simple Connect* is an application installed by Yaskawa America to manage peripheral equipment in a robotic work cell.
It is primarily installed on robots that were purchased with an arc-welding configuration.
This application is not compatible with MotoROS2 at this time.

**Work-around**: none at this time. Simple Connect must be removed to run MotoROS2.

Memory leak
===========

**Description**: there is a small memory leak which occurs each time the micro-ROS Agent disconnects from the controller.

**Work-around**: none at this time.
The issue is being investigated.

Some group combinations won’t publish data
==========================================

**Description**: it has been observed that an R1+R2+S1 system will not publish data ``/joint_states`` or ``/tf``.
All other topics and services work as expected.

**Work-around**: such a multi-group system would need to be broken up into a independent systems.
(E.g. ``R1+R2+S1`` would be broken up into ``R1+S1`` and another ``R1``).

The cause of this behavior is unknown.
The issue is being investigated.
