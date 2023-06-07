############
Installation
############

MotoROS2 binary
===============

Place the ``.out`` (main binary), ``.yaml`` (configuration), and ``.dat`` (I/O names) files on an external storage device: both Secure Digital (SD) and USB sticks can be used.
Insert the storage device into the robot’s programming pendant and refer to the following section.

YRC1000 and YRC1000micro
------------------------

Turn on the robot controller while holding the ``{Main Menu}`` key on the keypad to enter *Maintenance* mode.
You may release the key when you see the Yaskawa logo appear on the screen.

In *Maintenance* mode:

1. upgrade to *MANAGEMENT* security level by touching ``[System Info]``\ →\ ``[Security]`` (default password is all ``9``\ ’s)
2. touch ``[MotoPlus APL]``\ →\ ``[Device]`` to select either SD or USB memory type
3. touch ``[MotoPlus APL]``\ →\ ``[Load (User App)]`` to select and load ``motoros2_XXX.out``
4. touch ``[MotoPlus APL]``\ →\ ``[File List]`` and verify that MotoROS2 was properly installed and no other MotoPlus applications are currently loaded on the controller
5. touch ``[File]``\ →\ ``[Initialize]`` and select ``USER DEFINED FILES``
6. select ``SRAM RAM DRIVE`` and initialize it
7. rotate the pendant key-switch (upper left of pendant) fully counter-clockwise into ``TEACH`` mode
8. reboot the robot controller into regular mode

In *Normal Operation* mode:

1. you will get alarm ``8013 [0] Missing MotoROS2 cfg file``.
   touch ``[RESET]`` to clear the alarm
2. upgrade to *MANAGEMENT* security level by touching ``[System Info]``\ →\ ``[Security]`` (default password is all ``9``\ ’s)
3. touch ``[PARAMETER]``\ →\ ``[S2C]`` and set the following parameters:

   1. ``S2C1102 = 2``
   2. ``S2C1104 = 2``
   3. ``S2C1250 = 1``
   4. ``S2C1402 = 3``

4. touch ``[EX MEMORY]``\ →\ ``[Load]``
5. cursor to ``USER DEFINED FILES`` and press ``[SELECT]``
6. cursor to ``motoros2_config.yaml`` and press ``[SELECT]`` then ``[ENTER]``

If a custom INFORM job will be used:

1. touch ``[EX MEMORY]``\ →\ ``[Load]``
2. cursor to ``JOB`` and press ``[SELECT]``
3. cursor to your job file and press ``[SELECT]`` then ``[ENTER]``

Within 30 seconds of loading the configuration file, you should get alarm ``8001[10] Speed FB enabled, reboot now``. Reboot again and there should be no alarms.

If you receive any errors or alarms after rebooting, please refer to the `Troubleshooting <#troubleshooting>`__ section for information on how to remedy the issue.

The micro-ROS Agent
===================

The micro-ROS Agent acts as the transparent bridge between MotoROS2 and ROS 2.
As Micro-ROS applications can not directly communicate with ROS 2 RMWs, the Agent must always be running for MotoROS2 to function correctly.

There are two main ways to run the Agent: using a Docker image or by building it in a Colcon workspace.

Using the Docker image is recommended, as it’s less complicated than building with Colcon.

**Note**: either the Docker image *or* a from-source build is needed.
Choose one or the other.

Using Docker (Linux Only)
-------------------------

The command shown here starts the ``humble`` version of the ``micro-ros-agent`` Docker image.
However, always make sure to use a version of the Agent image which corresponds to the version of ROS 2 that is being used.

With ROS 2 Foxy, use ``microros/micro-ros-agent:foxy``.
With ROS 2 Humble, use ``microros/micro-ros-agent:humble``.

To start the Agent (on a machine with Docker already installed and setup to allow non-root access):

.. code:: shell

   docker run \
     -it \
     --rm \
     --net=host \
     microros/micro-ros-agent:humble \
       udp4 \
       --port 8888

**Note**: be sure to update the ``--port`` parameter to use the same value as was chosen for the ``agent_port_number`` configuration item in the ``motoros2_config.yaml``.

Using the ROS 2 package
-----------------------

The micro-ROS Agent can also be built as a ROS 2 package in a Colcon workspace.
This procedure is rather involved, so only do this if the pre-configured Docker image can not be used.

The following sections show how to build the Humble version of the Agent in a dedicated workspace (adapt the paths used below if a different workspace should be used instead).

Note: always make sure to use a version of the Agent which corresponds to the version of ROS 2 that is being used.
For ROS 2 Foxy, checkout the ``foxy`` branch.
For ROS 2 Humble, checkout the ``humble`` branch.

.. tab:: Linux (Debian/Ubuntu)

   This requires a working ROS 2 development environment (compiler, CMake, Git, Colcon, ``rosdep``, etc).
   
   In a terminal:
   
   .. code:: shell
   
      source /opt/ros/humble/setup.bash
      sudo apt update
      rosdep update --rosdistro=$ROS_DISTRO
      mkdir -p $HOME/micro_ros_agent_ws/src
      git clone \
        -b humble \
        https://github.com/micro-ROS/micro_ros_setup.git \
        $HOME/micro_ros_agent_ws/src/micro_ros_setup
      rosdep install \
        --from-paths $HOME/micro_ros_agent_ws/src \
        --ignore-src \
        -y
      cd $HOME/micro_ros_agent_ws
      colcon build
      source install/local_setup.bash
      ros2 run micro_ros_setup create_agent_ws.sh
      ros2 run micro_ros_setup build_agent.sh
   
   The Agent application will only need to be built *once*, unless it needs to be updated.
   
   Finally, to run the Agent:
   
   .. code:: shell
   
      source /opt/ros/humble/setup.bash
      source $HOME/micro_ros_agent_ws/install/local_setup.bash
      ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888
   
   **Note**: be sure to update the ``--port`` parameter to use the same value as was chosen for the ``agent_port_number`` configuration item in the ``motoros2_config.yaml``.

.. tab:: Windows

   This requires a working ROS 2 on Windows development environment (Visual Studio, CMake, Git, Colcon, etc).
   
   Open a Visual Studio *Developer Command Prompt*.
   
   Or open a regular command prompt and ``call "C:\Program Files\Microsoft Visual Studio\2022\Enterprise\VC\Auxiliary\Build\vcvars64.bat"``.
   If you are using a different version of Visual Studio, update the path to match.
   
   Now execute the following commands:
   
   .. code:: batch
   
      call "C:\path\to\ros2-humble\local_setup.bat"
      mkdir "%USERPROFILE%\micro_ros_agent_ws\src"
      git clone ^
        -b humble ^
        https://github.com/micro-ROS/micro_ros_msgs.git ^
        "%USERPROFILE%\micro_ros_agent_ws\src\micro_ros_msgs"
      git clone ^
        -b humble ^
        https://github.com/micro-ROS/micro-ROS-Agent.git ^
        "%USERPROFILE%\micro_ros_agent_ws\src\micro-ROS-Agent"
      cd "%USERPROFILE%\micro_ros_agent_ws"
      colcon build ^
        --merge-install ^
        --packages-up-to micro_ros_agent ^
        --cmake-args ^
        "-DUAGENT_BUILD_EXECUTABLE=OFF" ^
        "-DUAGENT_P2P_PROFILE=OFF" ^
        "--no-warn-unused-cli"
   
   The Agent application will only need to be built *once*, unless it needs to be updated.
   
   Finally, to run the Agent:
   
   .. code:: batch
   
      cd "%USERPROFILE%\micro_ros_agent_ws
      call install\setup.bat
      ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888
   
   **Note**: be sure to update the ``--port`` parameter to use the same value as was chosen for the ``agent_port_number`` configuration item in the ``motoros2_config.yaml``.

Verifying successful installation
=================================

After the final reboot of the controller, and after `starting the micro-ROS Agent <#the-micro-ros-agent>`__, the Agent should show MotoROS2 registering its publishers, services and action servers.

Note: if you are using ROS 2 Galactic, please first read `Only FastDDS is supported <#only-fastdds-is-supported>`__.

On a PC with a supported ROS 2 installation (ie: Foxy, Galactic (with FastDDS), Humble or Rolling):

1. open a new terminal
2. ``source`` the ROS 2 installation

Now run ``ros2 node list``.

Provided you have no other ROS 2 nodes running, you should see a single ``motoman_ab_cd_ef`` node listed, with ``ab_cd_ef`` being the last three bytes of the MAC address of the controller.
If you set ``node_name`` to something else in `the configuration <#updating-the-configuration>`__, you should of course see the expected node name listed instead.
