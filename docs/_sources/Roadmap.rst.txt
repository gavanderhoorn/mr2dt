
###################
Provisional roadmap
###################

This section gives a brief overview of features, enhancements and other tasks that are currently on the MotoROS2 roadmap.
Some of these may depend on external developments (ie: in ROS 2 or micro-ROS), or on Yaskawa internal development priorities.
As such, no statements are made about priorities or development schedule for any of these items.

The following items are on the MotoROS2 roadmap, and are listed here in no particular order:

- support DX200/FS100 controllers
- read/write of controller variables
- CRUD of INFORM job files (ie: create, retrieve, update, delete)
- starting/stopping INFORM jobs (other than ``INIT_ROS``)
- native (ie: Agent-less) communication
- support asynchronous motion / partial goals
- complete ROS parameter server support (there is currently no support for ``string``\ s in RCL)
- real-time position streaming interface (skipping MotoROS2's internal motion queue)
- Cartesian motion interfaces
- velocity control (based on ``mpExRcsIncrementMove(..)``)
- integration with ROS logging (``rosout``)
- publishing static transforms to ``tf_static``
- integrate a UI into the teach pendant / Smart Pendant
- add support for on-line trajectory replacement to the FJT action server (similar to the ROS1 `joint_trajectory_controller <http://wiki.ros.org/joint_trajectory_controller/UnderstandingTrajectoryReplacement>`_)
- integration of process control for peripherals attached to robot (welding, cutting, painting, etc)
