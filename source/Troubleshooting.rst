Troubleshooting
===============

Debug log client
----------------

This repository contains a debugging log client script in the ``tools`` folder.
It listens for debug output from MotoROS2, displays it on the console and logs it to a file in a session log (``*.txt``).

The log client is written in Python, and can be run on Windows and Linux.
To start it, either run it manually (in a ``cmd`` or Bash session) using ``python3 /path/to/debug_listener.py``, or on Windows by double-clicking the ``debug_listener.cmd`` file.

Note: you *must* have a recent version of Python 3 installed on the machine you intend to run the script on (the authors have tested the script with Python ``3.8``, but newer versions are also expected to be compatible).
On Windows, ``python.exe`` *must* be on the ``%PATH%`` (ie: you should be able to start ``python`` in a ``cmd`` shell from any location, without having to specify the path).

By default, logs will be written to the current working directory (``CWD``).
On Windows the ``.cmd`` file changes the ``CWD`` to the location of the script before starting it, so all logs will end up in the ``tools`` folder.

When you encounter an issue using MotoROS2, please start the debug client script and keep it running in the background while you reproduce the issue.
Attach the log it produces and a copy of the ``PANELBOX.LOG`` from the robot’s teach pendant to any support tickets you open on the `Issue tracker <https://github.com/yaskawa-global/motoros2/issues>`__.

MotoROS2 error and alarm codes
------------------------------

ERROR 3200: NOP or END instruction not found
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

The robot controller requires all files use Windows-style end-of-line terminator.
This is ``<carriage return><line feed>``.
Many common text editors in the Linux environment will automatically standardize all EOL terminators to be Unix-style; this removes the ``<carriage return>``, resulting in the error message.

*Solution:*
Open this job file in a text editor that allows you to use Windows-style EOL (``<0xD><0xA>`` or ``<CR><LF>``) or, given that the job is relatively small, manually recreate the job using the programming pendant.
Another alternative is to copy the ``.jbi`` and ``.dat`` files under Windows.

ERROR 4120: Concurrent I/O memory is full
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

This error can occur when attempting to load the ``motoros2_config.yaml`` configuration file.
It will occur if the SRAM drive was not initialized in the `installation procedure <../README.md#installation>`__.

Please note that this error can also occur if the SRAM storage is full.
This can occur if another application, such as Simple Connect, is using the storage.
The files for the other application must be removed to make room for MotoROS2.

*How to initialize SRAM drive:*

1. turn on the robot controller while holding the ``{Main Menu}`` key on the keypad to enter *Maintenance* mode.
2. upgrade to *MANAGEMENT* security level by touching ``[System Info]``\ →\ ``[Security]`` (default password is all ``9``\ ’s)
3. touch ``[File]``\ →\ ``[Initialize]`` and select ``USER DEFINED FILES``
4. select ``SRAM RAM DRIVE`` and initialize it

Alarm: 1020[5]
~~~~~~~~~~~~~~

*Example:*

.. code:: text

   ALARM 1020
    MOTOPLUS APPLICATION LOAD ERROR
   [5]

*Solution:*
This alarm occurs in conjunction with ``1050`` and ``4207``.
It is caused due to missing symbols when building from source.

Make sure the MotoROS2 binary is linked against all the required libraries.
The provided Visual Studio solution should be set up correctly.
Pay special attention to any errors or warnings displayed by Visual Studio as part of the build process.

If the error persists, you may need to upgrade the robot controller system software.
Please contact Yaskawa technical support for assistance with upgrading the controller.

Alarm: 1020[6]
~~~~~~~~~~~~~~

.. code:: text

   ALARM 1020
    MOTOPLUS APPLICATION LOAD ERROR
   [6]

*Solution:*
This alarm occurs in conjunction with ``1050`` and ``4207``.
There are two possible causes for this alarm.

1. Corruption of the MotoROS2 binary
2. Using MotoROS2 binary built for the wrong controller model

To remedy 1: download a fresh copy of the correct MotoROS2 binary and verify integrity of the file.

*Note*: make sure to check the binary copied to the removable medium (ie: usb stick or SD card), as corruption of files during transfer to such a medium is also possible.

To remedy 2: make sure that you are downloading the binary for your controller model.
For example, the YRC1000 binary will not run on an DX200.

Alarm: 1050[1]
~~~~~~~~~~~~~~

*Example:*

.. code:: text

   ALARM 1050
    SET-UP PROCESS ERROR(SYSCON)
   [1]

*Solution:*
If this alarm occurs in conjunction with ``1020`` (subcode ``5`` or ``6``) and ``4207`` (subcode ``1101``), please refer to `Alarm: 1020[5 - 6] <#alarm-10205>`__.

Alarm: 4207[1101]
~~~~~~~~~~~~~~~~~

*Example:*

.. code:: text

   ALARM 4207
    SYSTEM ERROR(MOTION)
   [1101]

*Solution:*
If this alarm occurs in conjunction with ``1020`` (subcode ``5`` or ``6``) and ``1050`` (subcode ``1``), please refer to `Alarm: 1020[5 - 6] <#alarm-10205>`__.

Alarm: 4430[6]
~~~~~~~~~~~~~~

*Example:*

.. code:: text

   ALARM 4430
    CPU COMMUNICATION ERROR
   [6]

*Solution:*
This alarm occurs if you run the DX100 version of the ``INIT_ROS`` Inform job on a non-DX100 controller.
Please delete the ``INIT_ROS`` job from your pendant and replace it with the version for all other controllers.

Alarm: 4997[4]
~~~~~~~~~~~~~~

*Example:*

.. code:: text

   ALARM 4997
    M-SAF DATA CRC UNMATCH
   [4]
   ALARM 8001
    Speed FB enabled, reboot now.
   [10]

*Solution:*
This alarm occurs if the FSU is enabled when installing MotoROS2.
The MotoROS2 driver attempts to enable the *Speed Feedback* parameters, but is unable to change the required parameters due to FSU settings.
You must temporarily disable the CRC check for the *Speed Feedback* update to complete.

Touch ``[RESET]`` to clear the active alarm.
Switch the pendant to TEACH mode.
Then, from the main menu, touch ``[System]``\ →\ ``[Security]`` and upgrade to *SAFETY* security level (the default password is all ``5``\ s).
Then touch ``[Setup]``\ →\ ``[Function Enable]``.
Navigate to *SAVE DATA CRC CHECK FUNC (FSU)*.
Set this feature to *INVALID* and reboot the controller in normal mode.

After rebooting, you should receive ``Alarm 8001 [10]`` (“Speed FB enabled, reboot now”).
Now *Speed Feedback* has been permanently enabled.

Reboot once more and follow the steps above to re-enable the *SAVE DATA CRC CHECK FUNC (FSU)*.

Alarm: 8001[10]
~~~~~~~~~~~~~~~

*Example:*

.. code:: text

   ALARM 8001
    Speed FB enabled, reboot now
   [10]

*Solution:*
This alarm is a one-time notice that feedback speed data has been automatically enabled.
This information will automatically be included with the feedback position data that is published as ``JointState`` messages.
When the alarm occurs, reboot the robot controller.
You should not see this alarm again.

Alarm: 8003[100 - 111]
~~~~~~~~~~~~~~~~~~~~~~

*Example:*

.. code:: text

   ALARM 8003
    MotoROS2: Controller cfg invalid
   [100]

*Solution:*
Your robot controller requires internal configuration changes to support the MotoROS2 driver.

For YRC1000 and YRC1000micro: ensure the controller is updated to at least ``YAS2.80.00-00`` (for YRC1000) and ``YBS2.31.00-00`` (for YRC1000micro).
If the system software version is below this, please contact Yaskawa Motoman for assistance with upgrading the controller.

Then boot the controller into *Maintenance* mode by holding ``{Main Menu}`` on the keypad.
Touch ``[System Info]``\ →\ ``[Security]`` and upgrade to *MANAGEMENT* security level.
Touch ``[System Info]``\ →\ ``[Setup]`` then select ``OPTION FUNCTION``.
Cursor down to ``MOTOMAN DRIVER`` and set this to ``USED``.
Now reboot the robot controller.

Alarm: 8003[1]
~~~~~~~~~~~~~~

*Example:*

.. code:: text

   ALARM 8003
    MotoROS2 Cfg: Set RS000=2
   [1]

*Solution:*
An internal parameter is not set properly in the robot controller.
Touch ``[System Info]``\ →\ ``[Security]`` and upgrade to *MANAGEMENT* security level.
Then touch ``[Parameter]``\ →\ ``[RS]`` and set the value of ``RS000 = 2``.
Then reboot your robot controller.

Alarm: 8003[2]
~~~~~~~~~~~~~~

*Example:*

.. code:: text

   ALARM 8003
    MotoROS2 Cfg: Set S2C541=0
   [2]

*Solution:*
An internal parameter is not set properly in the robot controller.
Touch ``[System Info]``\ →\ ``[Security]`` and upgrade to *MANAGEMENT* security level.
Then touch ``[Parameter]``\ →\ ``[S2C]`` and set the value of ``S2C541 = 0``.
Then reboot your robot controller.

Alarm: 8003[3]
~~~~~~~~~~~~~~

*Example:*

.. code:: text

   ALARM 8003
    MotoROS2 Cfg: Set S2C542=0
   [3]

*Solution:*
An internal parameter is not set properly in the robot controller.
Touch ``[System Info]``\ →\ ``[Security]`` and upgrade to *MANAGEMENT* security level.
Then touch ``[Parameter]``\ →\ ``[S2C]`` and set the value of ``S2C542 = 0``.
Then reboot your robot controller.

Alarm: 8003[4]
~~~~~~~~~~~~~~

*Example:*

.. code:: text

   ALARM 8003
    MotoROS2 Cfg: Set S2C1100=1
   [4]

*Solution:*
An internal parameter is not set properly in the robot controller.
Touch ``[System Info]``\ →\ ``[Security]`` and upgrade to *MANAGEMENT* security level.
Then touch ``[Parameter]``\ →\ ``[S2C]`` and set the value of ``S2C1100 = 1``.
Then reboot your robot controller.

Alarm: 8003[5]
~~~~~~~~~~~~~~

*Example:*

.. code:: text

   ALARM 8003
    MotoROS2 Cfg: Set S2C1103=2
   [5]

*Solution:*
An internal parameter is not set properly in the robot controller.
Touch ``[System Info]``\ →\ ``[Security]`` and upgrade to *MANAGEMENT* security level.
Then touch ``[Parameter]``\ →\ ``[S2C]`` and set the value of ``S2C1103 = 2``.
Then reboot your robot controller.

Alarm: 8003[6]
~~~~~~~~~~~~~~

*Example:*

.. code:: text

   ALARM 8003
    MotoROS2 Cfg: Set S2C1117=1
   [6]

*Solution:*
An internal parameter is not set properly in the robot controller.
Touch ``[System Info]``\ →\ ``[Security]`` and upgrade to *MANAGEMENT* security level.
Then touch ``[Parameter]``\ →\ ``[S2C]`` and set the value of ``S2C1117 = 1``.
Then reboot your robot controller.

Alarm: 8003[7]
~~~~~~~~~~~~~~

*Example:*

.. code:: text

   ALARM 8003
    MotoROS2 Cfg: Set S2C1119=0 or 2
   [7]

*Solution:*
An internal parameter is not set properly in the robot controller.
Touch ``[System Info]``\ →\ ``[Security]`` and upgrade to *MANAGEMENT* security level.
Then touch ``[Parameter]``\ →\ ``[S2C]`` and set the value of ``S2C1119``.
A value of ``2`` will enable the telnet option to see any output messages from the MotoROS2 driver.
A value of ``0`` will disable the telnet option.
Reboot your robot controller after changing this parameter.

Alarm: 8003[8]
~~~~~~~~~~~~~~

*Example:*

.. code:: text

   ALARM 8003
    MotoROS2 not compatible with PFL
   [8]

or:

.. code:: text

   ALARM 8003
    MotoROS2 not compatible with HC10
   [8]

*Solution:*
Old versions of the MotoROS2 driver are not compatible with the human-collaborative (HC) robots.
You must update to v1.9.0 or newer.
Reboot the robot controller while holding ``{Main Menu}`` on the keypad to enter *Maintenance* mode.
Touch ``[System Info]``\ →\ ``[Security]`` and upgrade to *MANAGEMENT* security level.
Then touch ``[MotoPlus Apl]``\ →\ ``[Delete]``.
Select the ``MotoROS2.out`` file and press ``{Enter}`` to confirm removal of the MotoROS2 driver.
Now follow the installation tutorial to install the latest version.

Additionally, the robot controller must meet a minimum version of system software.
For YRC1000, the controller must have ``YAS2.80.00-00`` or higher.
For YRC1000micro, the controller must have ``YBS2.31.00-00`` or higher.
Please contact Yaskawa technical support for assistance in upgrading the controller software.

Alarm: 8003[9]
~~~~~~~~~~~~~~

*Example:*

.. code:: text

   ALARM 8003
    MotoROS2 not compatible with HC10
   [9]

*Solution:*
Old versions of the MotoROS2 driver are not compatible with the human-collaborative (HC) robots.
You must update to v1.9.0 or newer.
Reboot the robot controller while holding ``{Main Menu}`` on the keypad to enter *Maintenance* mode.
Touch ``[System Info]``\ →\ ``[Security]`` and upgrade to *MANAGEMENT* security level.
Then touch ``[MotoPlus Apl]``\ →\ ``[Delete]``.
Select the ``MotoROS2.out`` file and press ``{Enter}`` to confirm removal of the MotoROS2 driver.
Now follow the installation tutorial to install the latest version.

Additionally, the robot controller must meet a minimum version of system software.
For YRC1000, the controller must have ``YAS2.80.00-00`` or higher.
For YRC1000micro, the controller must have ``YBS2.31.00-00`` or higher.
Please contact Yaskawa technical support for assistance in upgrading the controller software.

Alarm: 8003[11]
~~~~~~~~~~~~~~~

*Example:*

.. code:: text

   ALARM 8003
    HC/Convey trking not compatible
   [11]

*Solution:*
The Yaskawa Conveyor Tracking function is not compatible with HC robots.
Please contact Yaskawa technical support to have Conveyor Tracking function disabled.

Alarm: 8010[2]
~~~~~~~~~~~~~~

*Example:*

.. code:: text

   ALARM 8010
    FAILED TO CREATE TASK
   [2]

*Solution:*

This alarm is caused by having another MotoPlus application installed on the system which is using task-priority level ``MP_PRI_IP_CLK_TAKE``.
There can only be one ``MP_PRI_IP_CLK_TAKE`` task installed on the system, and it is required for ``MotoROS2``.

Either remove the additional MotoPlus application(s) installed on the controller, or modify the source code of the additional application so that it doesn’t use ``MP_PRI_IP_CLK_TAKE``.

Alarm: 8010[xx]
~~~~~~~~~~~~~~~

*Example:*

.. code:: text

   ALARM 8010
    FAILED TO CREATE TASK
   [0]

*Solution:*

1. Check the `setup instructions <../README.md#installation>`__ to ensure that ``MotoPlus FUNC.`` and ``MOTOMAN DRIVER`` have been enabled.
2. Verify there are no other MotoPlus applications (``.out`` file) installed on the robot controller.
3. Contact Yaskawa support to request that they enable additional MotoPlus tasks.

Alarm: 8011[xx]
~~~~~~~~~~~~~~~

*Example:*

.. code:: text

   ALARM 8011
    MotoROS2: Fatal Error
   [0]

*Solution:*
Check the `setup instructions <../README.md#installation>`__ to ensure that the robot controller is configured properly.

If the behavior persists, save a copy of the debug-listener script output and the ``PANELBOX.LOG`` from the robot’s teach pendant.
Open a new issue on the `Issue tracker <https://github.com/yaskawa-global/motoros2/issues>`__, describe the problem and attach ``PANELBOX.LOG`` and the debug log to the issue.
Include a verbatim copy of the alarm text as seen on the teach pendant (alarm number and ``[subcode]``).

Alarm: 8011[15]
~~~~~~~~~~~~~~~

*Example:*

.. code:: text

   ALARM 8011
    Domain ID (x) invalid
   [15]

*Solution:*
The ``ros_domain_id`` key must be configured in the ``motoros2_config.yaml`` configuration file.
The value must be between ``0`` and ``101``.

After correcting the configuration, the `changes will need to be propagated to the Yaskawa controller <../README.md#updating-the-configuration>`__.

Alarm: 8011[16]
~~~~~~~~~~~~~~~

*Example:*

.. code:: text

   ALARM 8011
    Missing Agent IP/Port
   [16]

*Solution:*
The ``agent_ip_address`` and ``agent_port_number`` keys must be configured in the ``motoros2_config.yaml`` configuration file.
This must be the IP address of the PC that runs the micro-ROS Agent application.
The port must match the number that was used when `starting the agent <../README.md#the-micro-ros-agent>`__ (default is ``8888``).

After correcting the configuration, the `changes will need to be propagated to the Yaskawa controller <../README.md#updating-the-configuration>`__.

Alarm: 8011[17]
~~~~~~~~~~~~~~~

*Example:*

.. code:: text

   ALARM 8011
    Agent IP on wrong subnet
   [17]

*Solution:*
The ``agent_ip_address`` key in the ``motoros2_config.yaml`` configuration file is an address that is not reachable by the robot controller.

Options:

1. Modify the ``agent_ip_address`` key and specify an IP address that is on the robot’s subnet.
   Now follow the instructions `to propagate the changes to the Yaskawa controller <../README.md#updating-the-configuration>`__.
2. Modify the robot controller’s IP so it is on the Agent’s subnet.
3. Modify the robot controller’s network settings to add a gateway which can reach the Agent’s IP address.

Refer to the relevant Yaskawa Motoman documentation for information on how to change the controller’s network configuration.

Alarm: 8011[19]
~~~~~~~~~~~~~~~

*Example:*

.. code:: text

   ALARM 8011
    Must enable ETHERNET function
   [19]

*Solution:*
The ETHERNET function must be enabled for one (or both) LAN interface in the robot controller.
Please contact your local Yaskawa representative to request this function.

Alarm: 8011[20]
~~~~~~~~~~~~~~~

*Example:*

.. code:: text

   ALARM 8011
    MotoROS2 - Multiple Instances
   [20]

*Solution:*
Multiple instances of the MotoROS2 MotoPlus application have been detected.
Only one instance may run on a single robot controller.

1. turn on the robot controller while holding the ``{Main Menu}`` key on the keypad to enter *Maintenance* mode.
2. touch ``[MotoPlus APL]``\ →\ ``[Delete]`` and remove any extra ``.out`` files that have been installed.
3. check the ``CN102`` USB port on the CPU board inside the controller cabinet. If there are any ``.out`` files on the root of this drive, delete them.

Alarm: 8011[21]
~~~~~~~~~~~~~~~

*Example:*

.. code:: text

   ALARM 8011
    Must specify node name
   [21]

*Solution:*
The ``node_name`` key must be configured in the ``motoros2_config.yaml`` configuration file.
The name must not be blank.

After correcting the configuration, the `changes will need to be propagated to the Yaskawa controller <../README.md#updating-the-configuration>`__.

Alarm: 8011[22]
~~~~~~~~~~~~~~~

*Example:*

.. code:: text

   ALARM 8011
    Must specify INFORM job name
   [22]

*Solution:*
The ``inform_job_name`` key must be configured in the ``motoros2_config.yaml`` configuration file.
The name must not be blank.

After correcting the configuration, the `changes will need to be propagated to the Yaskawa controller <../README.md#updating-the-configuration>`__.

Alarm: 8012[xx]
~~~~~~~~~~~~~~~

*Example:*

.. code:: text

   ALARM 8012
    OUT OF MEMORY
   [0]

*Solution:*
Verify there are no other MotoPlus applications (``.out`` file) installed on the robot controller.

If the behavior persists, save a copy of the debug-listener script output and the ``PANELBOX.LOG`` from the robot’s teach pendant.
Open a new issue on the `Issue tracker <https://github.com/yaskawa-global/motoros2/issues>`__, describe the problem and attach ``PANELBOX.LOG`` and the debug log to the issue.
Include a verbatim copy of the alarm text as seen on the teach pendant (alarm number and ``[subcode]``).

Alarm: 8013[0]
~~~~~~~~~~~~~~

*Example:*

.. code:: text

   ALARM 8013
    Missing MotoROS2 cfg file
   [0]

*Solution:*
Follow the `setup instructions <../README.md#installation>`__ to load the ``motoros2_config.yaml`` configuration file.
Be sure to follow the steps for initializing SRAM and setting the ``S2C`` parameters.

Alarm: 8013[1]
~~~~~~~~~~~~~~

*Example:*

.. code:: text

   ALARM 8013
    Fail to copy config file
   [1]
   ALARM 8013
    Set S2C1102=2; Init SRAMDRV.DAT
   [1]

*Solution:*
Set robot parameter ``S2C1102 = 2`` and follow the `setup instructions <../README.md#installation>`__ to initialize ``SRAMDRV.DAT``.

Alarm: 8013[2]
~~~~~~~~~~~~~~

*Example:*

.. code:: text

   ALARM 8013
    Fail to copy config file
   [2]
   ALARM 8013
    Initialize SRAMDRV.DAT
   [2]

*Solution:*
Follow the `setup instructions <../README.md#installation>`__ to initialize ``SRAMDRV.DAT``.

Alarm: 8013[3]
~~~~~~~~~~~~~~

*Example:*

.. code:: text

   ALARM 8013
    Invalid BOOL in motoros2_config
   [3]

*Solution:*
A key in the ``motoros2_config.yaml`` configuration file is set to an invalid value.
Boolean values should be set to either ``true`` or ``false``.

After correcting the configuration, the `changes will need to be propagated to the Yaskawa controller <../README.md#updating-the-configuration>`__.

Alarm: 8013[4]
~~~~~~~~~~~~~~

*Example:*

.. code:: text

   ALARM 8013
    Invalid QOS in motoros2_config
   [4]

*Solution:*
A key in the ``motoros2_config.yaml`` configuration file is set to an invalid value.
QoS values should be set to either ``sensor_data`` or ``default``.

After correcting the configuration, the `changes will need to be propagated to the Yaskawa controller <../README.md#updating-the-configuration>`__.

Alarm: 8013[5]
~~~~~~~~~~~~~~

*Example:*

.. code:: text

   ALARM 8013
    Invalid executor_sleep_period
   [5]

*Solution:*
A the ``executor_sleep_period`` key in the ``motoros2_config.yaml`` configuration file is set to an invalid value.
This must be set to an integer value between ``1`` and ``100`` milliseconds.

After correcting the configuration, the `changes will need to be propagated to the Yaskawa controller <../README.md#updating-the-configuration>`__.

Alarm: 8013[6]
~~~~~~~~~~~~~~

*Example:*

.. code:: text

   ALARM 8013
    Invalid fb_publisher_period
   [6]

*Solution:*
A the ``feedback_publisher_period`` key in the ``motoros2_config.yaml`` configuration file is set to an invalid value.
This must be set to an integer value between ``1`` and ``100`` milliseconds.

After correcting the configuration, the `changes will need to be propagated to the Yaskawa controller <../README.md#updating-the-configuration>`__.

Alarm: 8013[7]
~~~~~~~~~~~~~~

*Example:*

.. code:: text

   ALARM 8013
    Invalid status_monitor_period
   [7]

*Solution:*
The ``controller_status_monitor_period`` key in the ``motoros2_config.yaml`` configuration file is set to an invalid value.
This must be set to an integer value between ``1`` and ``100`` milliseconds.

After correcting the configuration, the `changes will need to be propagated to the Yaskawa controller <../README.md#updating-the-configuration>`__.

Alarm: 8013[8] or [9]
~~~~~~~~~~~~~~~~~~~~~

*Example:*

.. code:: text

   ALARM 8013
    Too many remap rules
   [8]

*Solution:*
The ``motoros2_config.yaml`` configuration file contains too many remap rules.
A maximum of 16 rules may be specified.

After correcting the configuration, the `changes will need to be propagated to the Yaskawa controller <../README.md#updating-the-configuration>`__.

Alarm: 8013[10] or [11]
~~~~~~~~~~~~~~~~~~~~~~~

*Example:*

.. code:: text

   ALARM 8013
    Invalid remap rule format
   [10]

*Solution:*
The ``motoros2_config.yaml`` configuration file contains a remap rule that has been specified in the wrong format.

Example format: ``remap_rules: "joint_states:=my_joint_states read_single_io:=io/read_single"``

After correcting the configuration, the `changes will need to be propagated to the Yaskawa controller <../README.md#updating-the-configuration>`__.

Alarm: 8014[0]
~~~~~~~~~~~~~~

*Example:*

.. code:: text

   ALARM 8014
    MotoROS2 failed to validate job
   [0]

*Solution:*
Follow the `setup instructions <../README.md#installation>`__ to ensure that all robot parameters are set correctly.
In particular, be sure to check ``S2C1102 = 2``.

Alarm: 8014[1]
~~~~~~~~~~~~~~

*Example:*

.. code:: text

   ALARM 8014
    Invalid MotoROS2 job detected
   [1]

*Solution:*
An invalid MotoROS2 INFORM job was detected (the job contains INFORM statements MotoROS2 did not expect).

If MotoROS2 should use the custom job, instead of a default, auto-generated one, please update the ``allow_custom_inform_job`` field in the yaml configuration file.
After correcting the configuration, the `changes will need to be propagated to the Yaskawa controller <../README.md#updating-the-configuration>`__.

If there is no need to use a customised job, delete the existing job and reboot to allow the default job to be generated.

If the alarm is posted again after restarting the controller, make sure to allow sufficient time for the controller to properly delete the job (flushing changes to the file system may take some time).
Allow for at least 20 seconds between deleting the job and restarting the controller.

Alarm: 8014[2]
~~~~~~~~~~~~~~

*Example:*

.. code:: text

   ALARM 8014
    MotoROS2 failed to generate job
   [2]

*Solution:*
There was a failure when generating the default INFORM job.
Please obtain the standard job from the Github repository.

Alarm: 8014[3]
~~~~~~~~~~~~~~

*Example:*

.. code:: text

   ALARM 8014
    MotoROS2 failed to load job
   [3]

*Solution:*
There was a failure when generating the default INFORM job.
Please obtain the standard job from the Github repository and load it using the teach pendant.
