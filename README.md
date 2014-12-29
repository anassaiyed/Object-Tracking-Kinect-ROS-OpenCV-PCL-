Object-Tracking-Kinect-ROS-OpenCV-PCL-
======================================

Track object in XYZ using multiple depth cameras

Compiling and installing k4w library: Follow the instructions on http://openkinect.org/wiki/Getting_Started for Ubuntu.

    Go to build directory and type "cmake ..".
    "make".
    "sudo make install".
    To uninstall, "sudo make uninstall".

In ROS, install the freenect-stack. Replace all the libfreenect library (.so) and binary files in the opt/ros/hydro directory with the ones we just compiled.

ROS libfreenect-stack should now work with kinect for windows.
