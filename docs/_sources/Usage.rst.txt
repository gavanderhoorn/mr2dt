#####
Usage
#####

Basic usage with ROS
====================

Note: if you are using ROS 2 Galactic, please first read `Only FastDDS is supported <#only-fastdds-is-supported>`__.

As MotoROS2 uses micro-ROS, you must always make sure to first `start the micro-ROS Agent <#the-micro-ros-agent>`__.
After registration with the Agent, MotoROS2 behaves like any other ROS 2 node.

For initial discovery of the ROS API, we recommend using the `ros2 node <https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Nodes/Understanding-ROS2-Nodes.html>`__, `ros2 topic <https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Topics/Understanding-ROS2-Topics.html>`__, `ros2 service <https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Services/Understanding-ROS2-Services.html>`__ and `ros2 action <https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Actions/Understanding-ROS2-Actions.html>`__ commandlets.
Refer also to the `ROS API <#ros-api>`__ section for more information on the various topics, services and actions MotoROS2 supports.

In order to be able to interact with MotoROS2 (either using the command line tools or from ROS 2 nodes), the message, service and action definitions must first be made available.

MotoROS2 uses interface definitions from:

-  ``control_msgs``
-  ``industrial_msgs``
-  ``motoros2_interfaces``
-  ``sensor_msgs``
-  ``std_srvs``
-  ``tf2_msgs``

Except ``industrial_msgs`` and ``motoros2_interfaces``, these are all standard ROS 2 packages.

The `motoros2_client_interface_dependencies <https://github.com/yaskawa-global/motoros2_client_interface_dependencies>`__ package has been created to facilitate installation of all required interface packages.
Please refer to the readme of ``motoros2_client_interface_dependencies`` for information on how to download, configure, build and install the necessary packages in a Colcon workspace.

After installing ``motoros2_client_interface_dependencies``, you should be able to ``ros2 topic echo`` all topics MotoROS2 publishes to and ``ros2 service call`` all services MotoROS2 serves.

Even though the action server can also be interacted with from the command line using ``ros2 action send_goal``, we do not recommend this.
Instead, write a ``FollowJointTrajectory`` action *client* script or use a motion planning environment which is capable of interfacing with ``FollowJointTrajectory`` action *servers*, such as MoveIt.

Commanding motion
=================

The ROS API of MotoROS2 for commanding motion is similar to that of ``motoman_driver`` (with MotoROS1), and client applications are recommended to implement a similar flow of control to keep track of the state of the robot before, during and after trajectory and motion execution.

The following provides a high-level overview of the behaviour a client application should implement to successfully interact with the `FollowJointTrajectory <doc/ros_api.md#follow_joint_trajectory>`__ action server offered by MotoROS2.
While not all steps are absolutely necessary, checking for errors and monitoring execution progress facilitates achieving robust execution and minimises unexpected behaviour in both client and server.

To submit a trajectory goal for execution by an idle MotoROS2 (so no active ROS nor INFORM motion):

1.  create a ``FollowJointTrajectory`` action client, pointing it to the `follow_joint_trajectory <doc/ros_api.md#follow_joint_trajectory>`__ action server of MotoROS2
2.  retrieve a message from the `robot_status <doc/ros_api.md#robot_status>`__ topic to verify the robot is idle (inspect the ``RobotStatus::in_motion`` field) and store the current ``JointState`` by retrieving a message from the `joint_states <doc/ros_api.md#joint_states>`__ topic
3.  generate (using a motion planner for instance) or construct a ``trajectory_msgs/JointTrajectory`` message, making sure to use the state stored in step 2 as the start state (ie: the first ``JointTrajectoryPoint``).
    Using correct start states for trajectories is absolutely required, and MotoROS2 will reject any trajectory which does not start at the current state
4.  retrieve a message from the `robot_status <doc/ros_api.md#robot_status>`__ topic and inspect the ``in_error`` and ``error_code`` fields
5.  if there are no active errors, go to step 6.
    If there are active errors, remedy the cause, call the `reset_error <doc/ros_api.md#reset_error>`__ service and go to step 4
6.  call the `start_traj_mode <doc/ros_api.md#start_traj_mode>`__ service, to put MotoROS2 into its trajectory execution mode.
    Inspect the ``result_code`` of the service response.
    If MotoROS2 reported success, proceed to step 7; if instead, it reported an error, remedy the cause and go to step 4
7.  construct a ``FollowJointTrajectory`` goal and store the trajectory created in step 3 in it
8.  submit the goal to the server, using the client created in step 1
9.  wait for MotoROS2 to accept the goal.
    After acceptance, keep monitoring goal state while MotoROS2 executes it (note: this is default behaviour of the ROS 2 action client. It will call registered callbacks and forward state received from MotoROS2 to user code automatically)
10. if the goal is ``aborted`` or ``rejected``, inspect the ``error_code`` field of the returned action result.
    Otherwise wait for MotoROS2 to report completion of the goal.
    As a final check, inspect the ``error_code`` field of the result to ascertain execution was successful (refer to `follow_joint_trajectory <doc/ros_api.md#follow_joint_trajectory>`__ for information on how to decode the field’s value)
11. if there are more trajectories to execute, return to step 2.
    Otherwise call the `stop_traj_mode <doc/ros_api.md#stop_traj_mode>`__ service to exit MotoROS2’s trajectory execution mode

Interaction with the *point streaming* interface (MotoROS2 ``0.0.15`` and newer) would be similar, although no ``FollowJointTrajectory`` action client would be created, no goals would be submitted and monitoring robot status would be done purely by subscribing to the `robot_status <doc/ros_api.md#robot_status>`__ topic (instead of relying on an action client to report trajectory execution status).

With MoveIt
===========

Make sure to first have read `Basic usage with ROS <#basic-usage-with-ros>`__ and `Commanding motion <#commanding-motion>`__.

MotoROS2 exposes the required ``JointState`` and ``FollowJointTrajectory`` interfaces for MoveIt to be able to control motion.

For this to work, a MoveIt configuration package needs to be updated to send ``FollowJointTrajectory`` action goals to the correct action server, which requires changes to MoveIt’s *controller configuration*.
This part of the configuration can typically be found in a file called ``moveit_controllers.yaml``, which is normally located in the ``config`` sub directory of a MoveIt configuration package.

The MoveIt defaults for subscribing to ``JointState`` messages should not need any changes (unless MotoROS2 has been configured with a non-default setting for the namespace (see below)).

The following snippet shows an example of how to update the ``controller_names`` and ``action_ns`` keys for a robot with a single group, 6 joints, with their default joint names and no namespacing configured on MotoROS2’s side:

.. code:: yaml

   ...

   controller_names:
     - follow_joint_trajectory

   follow_joint_trajectory:
     action_ns: ""
     type: FollowJointTrajectory
     default: true
     joints:
      - group_1/joint_1
      - group_1/joint_2
      - group_1/joint_3
      - group_1/joint_4
      - group_1/joint_5
      - group_1/joint_6

   ...

Here, ``follow_joint_trajectory`` is the name of the MotoROS2 action server, but used as the name of the MoveIt controller.
Note also the empty ``action_ns`` key.

Note: depending on whether MotoROS2 is configured to *namespace* its action server (see `Can MotoROS2 run in a namespace? <doc/faq.md#can-motoros2-run-in-a-namespace>`__), the ``action_ns`` and name of the controller will need to be updated.

In case `joint names have been changed from their defaults <doc/faq.md#can-names-of-joints-be-changed>`__, the names specified in the MoveIt configuration will also need to be updated.
