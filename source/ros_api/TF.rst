..
   SPDX-FileCopyrightText: 2023, Yaskawa America, Inc.
   SPDX-FileCopyrightText: 2023, Delft University of Technology

   SPDX-License-Identifier: CC-BY-SA-4.0


##########
Transforms
##########

Required TF transforms
======================

None.

Provided TF transforms
======================

world → base
------------

Transform from the origin of the Yaskawa *BF* to the origin of the current Yaskawa *RF*.

Note: the child frame ``base`` follows ROS-Industrial conventions, it is not the Yaskawa BF.

Note 2: this transform will not be correct for multi-robot setups (for example, an ``R1+R2`` configuration) until the robot group(s) have been calibrated.
See also `Incorrect transform tree origin with multi-robot setups <../README.md#incorrect-transform-tree-origin-with-multi-robot-setups>`__.

base → flange
-------------

Transform from Yaskawa *RF* to ROS-Industrial ``flange`` frame.

This frame’s origin coincides with the Yaskawa flange, but is rotated such that it always follows REP-103 (ie: relative to the link, X+ is forward, Y+ left, Z+ up, instead of Z+ forward).

Attaching EEF models to this frame now becomes straightforward, as the frame orientation is always the same across different robots (with different zero poses).

flange → tool0
--------------

Transform from the ROS-Industrial ``flange`` frame to the ROS-Industrial ``tool0`` frame (an “all zeros toolframe”).

This frame coincides with the location and orientation of an *unconfigured* tool file on the Yaskawa controller, and will never change (ie: it’s a static transform), not even if tool files are configured.

Note: this is not the same as Yaskawa’s *Tool No. 0* (ie: tool file 0).
Only if that tool file is still unconfigured (ie: all zeros) would ``tool0`` and tool file 0 coincide.

flange → tcp_N
--------------

Transform from ROS-Industrial ``flange`` to the currently active Yaskawa TCP.
