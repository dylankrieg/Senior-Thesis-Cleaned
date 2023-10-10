This directory contains a Webots world and a controller for interfacing with a UR5 robot arm that rearranges blocks in the canonical block world from classical AI.

I (Dylan) never incorporated the webots simulation with the full task-planning perception architecture in "Full-System".
It could be ported. 

I ran into issues using jupyter notebooks with Webots so it may need to be written in regular .py files which have worked with Webots.

The code here is simply an interface for running a simple routine that moves the UR5 between the block positions.
Future work might include adding the Magpie gripper instead of the ROBOTIQ.

