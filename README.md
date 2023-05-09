# Overview:
This repository contains a system that enable a Universal Robotics UR5 to assemble blocks procedurally based on a high-level task plan in a semi-structured world. All code is writted in Python with a ipython notebook interface. 

Blocks are manipulated using a 3D printed gripper (MAGPIE) (https://github.com/correlllab/MAGPIE).
The gripper has a palm-mounted Realsense D405 that is used for estimating block pose and generating a world model in PDDL.

# Dependencies:

All installation instructions assume use on Linux (Ubuntu). For platform-specific instructions view the instructions associated with each link.

Universal Robotics Real-Time Data Exchange (RTDE) Interface for Interfacing with UR5 (https://sdurobotics.gitlab.io/ur_rtde/installation/installation.html#quick-install):

1. $ sudo add-apt-repository ppa:sdurobotics/ur-rtde

2. $ sudo apt-get update

3. $ sudo apt install librtde librtde-dev

4. $ python3 -m pip install ur_rtde


Dynamixel SDK for Interacing with MAGPIE Servos:

1. $ python3 -m pip install dynamixel-sdk pyax12

2. sudo cp openCM.rules /etc/udev/rules.d/

3. sudo adduser $USER dialout

4. Logout or Restart

Spatial Maths for Manipulation of Coordinate Frames and Kinematics (https://petercorke.github.io/spatialmath-python/intro.html):

1. $ python3 -m pip install spatialmath-python


Ultralytics YoloV8 for Object Segmentation (https://docs.ultralytics.com/quickstart/)

1. Install ultralytics with version>=8.0.91

2. $ python3 -m pip install ultralytics


Open3D for 3D Visualization and Point Cloud Processing:

1. Install Open3D for Python with version>=0.16.0 (http://www.open3d.org/docs/0.16.0/getting_started.html)

2. $ pip install -U pip>=20.3

3. $ python3 -m pip install open3d


PyRealSense for Interfacing with D405:

1. Install RealSense SDK 2.0. (https://github.com/IntelRealSense/librealsense)

2. Install pyrealsense2 (https://pypi.org/project/pyrealsense2/)

$ python3 -m pip install pyrealsense


Py2PDDL for Interacing with Planning Domain Definition Language (PDDL) in Python:

1. $ python3 -m pip install git+https://github.com/remykarem/py2pddl#egg=py2pddl


Fast Downward for Solving PDDL Plans:

1. Clone fast downward to this directory

$ git clone https://github.com/aibasel/downward

2. Build fast downward

$ cd downward

$ ./build.py release 


Python Imaging Library (PIL) for image file access:

1. $ python3 -m pip install Pillow


Numpy for Math Operations (https://numpy.org/install/):

1. $ python3 -m pip install numpy


Recommended Optional Libraries for Feature-Building:

Robotics Toolbox for Python enables Kinematics for Manipulators (https://github.com/petercorke/robotics-toolbox-python)

1. $ python3 -m pip install roboticstoolbox-python

Swift Simulator for Robot Arm Kinematics and 3D Visualization (https://github.com/jhavl/swift)

1. $ python3 -m pip install swift-sim








