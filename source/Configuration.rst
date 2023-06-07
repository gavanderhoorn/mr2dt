
#############
Configuration
#############

First time configuration
========================

All MotoROS2 distribution ``.zip`` files come with a template ``.yaml`` file, which *must* be updated before deployment to a specific Yaskawa Motoman robot controller.

For more information on all configuration settings, please refer to the ``motoros2_config.yaml`` file in the release ``.zip``.

All fields have defaults, except the IP address and UDP port number at which MotoROS2 expects the micro-ROS Agent to be reachable.
These fields must be set to correct values by users, as otherwise MotoROS2 will not be able to communicate with the ROS 2 node graph.

Edit the template ``.yaml`` and change the ``agent_ip_address`` and ``agent_port_number`` to point to the host which will run the micro-ROS Agent application.
Review the rest of the configuration file and change values as necessary for your specific deployment.

Verifying YAML correctness
==========================

We recommend checking YAML syntax of the configuration file before copying it to the controller.

**Note**: we recommend using an off-line tool, to prevent information in the ``.yaml`` file from being submitted to an on-line service.
Users are of course free to use tools they prefer, but we’ll only document using ``yamllint`` (ie: an off-line tool) in this section.

``yamllint`` is written in Python, so works on Windows, Linux and OSX.
Many package managers (such as ``apt``, on Ubuntu and Debian) include it in their default distributions, but the versions available are generally too old, so the developers use a Python virtual environment:

.. code:: shell

   python3 -m venv $HOME/venv_yamllint
   source $HOME/venv_yamllint/bin/activate
   (venv_yamllint) pip3 install yamllint

MotoROS2 comes with a configuration file for `yamllint <https://yamllint.readthedocs.io/en/stable>`__ which facilitates checking ``motoros2_config.yaml`` with the settings the developers use.

To verify the file has been correctly edited:

.. code:: shell

   (venv_yamllint) cd /path/to/motoros2/config
   (venv_yamllint) yamllint -s . && echo "all ok"

For a correctly formatted file, this should print ``all ok``.

If ``yamllint`` prints warnings or errors, correct the offending line(s) and rerun ``yamllint``.

Updating the configuration
==========================

It may be necessary to update MotoROS2 configuration during or after initial deployment.

Controller software YAS4.70 or YBS3.02 or later
-----------------------------------------------

In *Normal Operation* mode:

1. touch ``[EX MEMORY]``\ →\ ``[Device]``
2. cursor to ``JOB & U.D. FILE LOAD OVERWRITE`` and verify it is set to ``VALID``.
   Change it to ``VALID`` if it isn’t.
3. touch ``[EX MEMORY]``\ →\ ``[Load]``
4. cursor to ``USER DEFINED FILES`` and press ``[SELECT]``
5. cursor to ``motoros2_config.yaml`` and press ``[SELECT]`` then ``[ENTER]``
6. touch ``[YES]`` to overwrite
7. reboot the robot controller

Older controller software
-------------------------

Due to the way the controller treats files, ``motoros2_config.yaml`` cannot be directly overwritten using the ``[EX MEMORY]`` menu.
Instead, MotoROS2 has a built-in mechanism which updates the controller’s copy of the ``.yaml`` file with a new version placed on a USB stick.

To update the configuration file on the controller:

1. place an updated version of the ``motoros2_config.yaml`` file in the root directory of a USB stick
2. power down the robot controller and open the cabinet
3. locate the USB port labelled ``CN102`` on the robot’s CPU board and insert the USB stick into it
4. power on the controller and wait for it to fully boot (ie: you see the regular UI on the teach pendant)
5. verify MotoROS2 has started
6. remove the USB stick from the controller

MotoROS2 should automatically parse the new file and update its internal configuration.
In case of errors in the configuration file, or incompatible settings, alarms and/or errors will be posted to the teach pendant.

After the new file is processed, it will be renamed on the USB stick.
The filename will be appended with the current date/time of the robot controller.
This is to show that the file was processed and maintain some historical record.

NOTE: the USB port on the teach pendant can not be used to update the configuration.
*Only* the USB port labelled ``CN102`` is checked by MotoROS2.

CF and SD cards are not supported when updating the configuration.

NOTE: When submitting support requests, please always get a ‘current’ copy of the configuration file from the teach pendant.
You should not rely on the historical configuration files and assume that is currently loaded into the controller.

To extract a copy of your current configuration from the teach pendant:

1. if necessary: restart the controller in *Normal* mode
2. touch ``[EX MEMORY]``\ →\ ``[SAVE]``
3. cursor to ``USER DEFINED FILES`` and press ``[SELECT]``
4. cursor to ``motoros2_config.yaml`` and press ``[SELECT]`` then ``[ENTER]``

Default QoS settings
====================

Publisher QoS
-------------

The default QoS profiles used for the topics MotoROS2 publishes to are listed in this table:

================ ============= =========================
Topic            Profile       Comment
================ ============= =========================
``joint_states`` *Sensor data* *Best-effort* reliability
``robot_status`` *Sensor data* Same
``tf``           *Default*     *Reliable* reliability
================ ============= =========================

Please refer to `About Quality of Service settings: QoS profiles <https://docs.ros.org/en/humble/Concepts/About-Quality-of-Service-Settings.html#qos-profiles>`__ in the general ROS 2 documentation for more information about these default profiles.

These values are based on tests with other ROS 2 components and applications and analyses of implementations of subscribers by the authors of MotoROS2 (examples include RViz2 and MoveIt2).
But these profiles can not be compatible with all possible combinations of subscribers, and thus have been made configurable.

If needed, update `the configuration <#configuration>`__ by changing the ``publisher_qos`` settings: ``joint_states``, ``robot_status`` and/or ``tf`` items.
Follow `Updating the configuration <#updating-the-configuration>`__ to propagate this change to MotoROS2.

Note: a default QoS profile for topics like ``joint_states`` is a topic for discussion in the ROS 2 community.
Some relevant discussions and issues: `QoS settings for /joint_states in Humble <https://answers.ros.org/question/399325>`__ (ROS Answers) and `ros-planning/moveit2#1190 <https://github.com/ros-planning/moveit2/issues/1190>`__.

Service server QoS
------------------

MotoROS2 uses the default ROS 2 / micro-ROS profile for all its service servers.
This is called *Services* in the ROS 2 documentation on QoS profiles.

QoS for services is currently not configurable.

Action server QoS
-----------------

MotoROS2 uses the default ROS 2 / micro-ROS profiles for all its action servers.
As actions use both topics as well as services, both the *Services* and the *Default* profile are used.

QoS for action servers is currently not configurable.

The action clients shipped by ROS 2 as part of the ``rclcpp`` and ``rclpy`` client libraries are by default compatible with MotoROS2.
