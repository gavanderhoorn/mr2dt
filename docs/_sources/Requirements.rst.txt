####################
General Requirements
####################

The following general requirements must be met in order to be able to use MotoROS2 with a Yaskawa Motoman robot controller:

-  controller series: YRC1000 or YRC1000micro
-  minimum versions of system software:

   -  ``YAS2.80.00-00`` for YRC1000
   -  ``YBS2.31.00-00`` for YRC1000micro

-  the controller must have a correctly configured network connection (either ``LAN2`` or ``LAN3``)
-  MotoPlus and Motoman-Driver must be enabled on the controller
-  ROS 2 version: Foxy, Galactic, Humble or Rolling.
   ROS 2 Iron Irwini is not yet supported.
-  Docker or a from-source build of the micro-ROS Agent
-  FastDDS as RMW (even when using ROS 2 Galactic)

Checking the system software version
====================================

To check the version of the system software:

1. touch ``{MAIN MENU}`` on the pendant keypad
2. touch ``[System Info]``\ →\ ``[Version]``

Look for the version number starting with ``YAS`` or ``YBS``.

Checking MotoPlus configuration
===============================

Use the following steps to verify MotoPlus has been correctly configured for MotoROS2, and the necessary settings are active:

1. boot the controller while holding ``{MAIN MENU}`` on the pendant keypad to enter *Maintenance* mode
2. upgrade to *MANAGEMENT* security level by touching ``[System Info]``\ →\ ``[Security]`` (default password is all ``9``\ ’s)
3. touch ``[System Info]``\ →\ ``[Setup]`` and select ``OPTION FUNCTION``
4. move to ``MotoPlus FUNC.``, make sure it is set to ``USED``. If it isn’t, set it to ``USED``
5. move cursor down to ``MOTOMAN DRIVER`` and make sure it is set to ``USED``. If it isn’t, set it to ``USED``
