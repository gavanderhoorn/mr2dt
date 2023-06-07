
#####################
Downloading the files
#####################


All MotoROS2 releases are available from the `Releases <https://github.com/yaskawa-global/motoros2/releases>`__ page.

Each release is a ``.zip``, which contains the main executable (``.out``), a template configuration file (``.yaml``) and the INFORM jobs (several ``.jbi``\ s).

Save the ZIP file to a convenient location.

Extracting the files
====================

Extract the ``.zip`` file somewhere to a temporary location.

Note that after installation, neither the ``.zip`` itself, nor any of the files in it are needed any more, as the driver node itself runs *on* the Yaskawa Motoman controller.

Verifying integrity of the download
===================================

Check the integrity of the downloaded binary to avoid loading corrupted or incomplete binaries onto the controller.

The example command below uses ``md5sum`` on Linux, but any utility which can calculate MD5 hashes could be used, on any OS.

To calculate the MD5 hash on Debian/Ubuntu for the main MotoROS2 binary, run the following commands (on other OS, use the appropriate commands or tools instead):

.. code:: shell

   $ cd /path/to/where/the/binary/was/saved
   $ md5sum -b motoros2_yrc1000_humble.out
   c5d0f2cce281ed1cb8194badaaffc511  motoros2_yrc1000_humble.out

Compare the output of ``md5sum`` when run against the binary downloaded in the previous section (`Downloading the files <#downloading-the-files>`__) with the values listed in the following table.
The values must match *exactly*.

.. tab:: YRC1000

   +-------------------+------------------------------------+-------------+--------------------------------------+
   | **ROS 2 Version** | **File**                           | **Version** | **MD5 hash**                         |
   +===================+====================================+=============+======================================+
   | Foxy              | ``motoros2_yrc1000_foxy.out``      | ``v0.1.0``  | ``eb3c028d0989b6cce2eb4d50a9f45001`` |
   +-------------------+------------------------------------+-------------+--------------------------------------+
   | Galactic          | ``motoros2_yrc1000_galactic.out``  | ``v0.1.0``  | ``78abcead2e2504109a49287648a9bc04`` |
   +-------------------+------------------------------------+-------------+--------------------------------------+
   | Humble            | ``motoros2_yrc1000_humble.out``    | ``v0.1.0``  | ``c5d0f2cce281ed1cb8194badaaffc511`` |
   +-------------------+------------------------------------+-------------+--------------------------------------+

.. tab:: YRC1000micro

   +-------------------+------------------------------------+-------------+--------------------------------------+
   | **ROS 2 Version** | **File**                           | **Version** | **MD5 hash**                         |
   +===================+====================================+=============+======================================+
   | Foxy              | ``motoros2_yrc1000u_foxy.out``     | ``v0.1.0``  | ``fe17a90ca6e4a86447e9206a273486f2`` |
   +-------------------+------------------------------------+-------------+--------------------------------------+
   | Galactic          | ``motoros2_yrc1000u_galactic.out`` | ``v0.1.0``  | ``080957841a010aee9b479d36b5f6c1b8`` |
   +-------------------+------------------------------------+-------------+--------------------------------------+
   | Humble            | ``motoros2_yrc1000u_humble.out``   | ``v0.1.0``  | ``6b5df5c2764924466903111d0f61502a`` |
   +-------------------+------------------------------------+-------------+--------------------------------------+


If the hash matches, proceed with the next section, `Configuration <#configuration>`__.

If the hash does not match, download the correct version from the `Releases <https://github.com/yaskawa-global/motoros2/releases>`__ page again (to exclude a failed download was the cause), extract it and run ``md5sum`` on the binary again.
If the hash still doesn’t match, report the issue on the `issue tracker <https://github.com/yaskawa-global/motoros2/issues>`__.
Be sure to describe which version of MotoROS2 was downloaded, from where, how it was extracted, and a verbatim copy-paste of the hashing tool command executed and the output received.

**Note**: please verify you ran ``md5sum`` against the ``.out`` file, not the ``.zip`` nor any other file included in the release.
