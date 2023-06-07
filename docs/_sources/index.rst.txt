:hide-toc:

################
Yaskawa MotoROS2
################

The MotoROS2 documentation.

WIP.


Quickstart
==========

-  check `prerequisites <#general-requirements>`__
-  download the `latest release <#download>`__ and extract onto USB stick
-  edit the `configuration file <#configuration>`__, at least Agent IP and port
-  insert USB stick into pendant
-  *Maintenance* mode: load the appropriate ``motoros2_yrc1000_*.out`` or ``motoros2_yrc1000u_*.out`` MotoPlus application `onto controller <#installation>`__
-  *Online* mode: load ``motoros2_config.yaml`` via ``USER DEFINED FILES``
-  start an instance of the `micro-ROS Agent <#the-micro-ros-agent>`__ on a PC
-  reboot controller
-  verify `MotoROS2 is running <#verifying-successful-installation>`__
-  read about `known issues and limitations <#known-issues--limitations>`__
-  `troubleshoot <#troubleshooting>`__ problems


License
=======

MotoROS2 is made available under the Apache 2.0 license.

MotoROS2 depends on various open-source projects which come with their own licenses.



.. toctree::
  :hidden:
  :caption: Deployment
  :glob:

  Requirements
  Downloading
  Configuration
  Installation
  Usage
  KnownIssuesAndLimitations
  Troubleshooting
  faq
  Roadmap

.. toctree::
  :hidden:
  :caption: ROS API
  :glob:

  ros_api/Actions
  ros_api/Topics
  ros_api/Services
  ros_api/TF

.. toctree::
   :hidden:
   :caption: Development
   :glob:

   building_from_source
