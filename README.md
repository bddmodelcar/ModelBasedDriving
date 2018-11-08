#### How to start program:
1. Compile (if needed): `catkin_make`
1. `source ./devel/setup.bash`
2. `roslaunch bdd bdd.launch`




#### How to install ZED Camera Wrapper for ROS
1. Download ZED SDK for TX1 (or TX2)
2. Find the downloaded .run file in Downloads folder and make it executable using `chmod +x`
3. Run it using `./ZED_SDK_LINUX_JTX1_JP...`
4. Accept all installation options
5. Follow other instruction for downloading and installing ZED ROS wrapper.
6. Execute `catkin_make zed-ros-wrapper` in your catkin directory
7. `source ./devel/setup.bash`
8. The wrapper ends up being a git repo so you can do `git pull` to update it.



#### Mandatory:
1. Add to `.bashrc` the following: `export PYTHONPATH=$PYTHONPATH:/usr/lib/python2.7/dist-packages` in order to run the ROS nodes
2. Nodes must be executable to work: make nodes executable using `chmod +x <file_name>.py` and by adding this at the top of the .py file: `#!/usr/bin/env python`



#### Helpful Tips / Troubleshooting:
1. source devel/setup.bash
2. ROS's built-in `sensor_msgs` package defines messages for commonly used sensors. The ZED camera uses `Image.msg` message type when publishing images. Its implementation can be found using `roscd sensor_msgs/msg`
3. Settings for the ZED camera are found at zed.launch, and the launch file(s) it includes. Also be sure to read its READMEs which are at multiple places in its directory tree.
4. When connecting the flash drive used for storing log files, it mounts itself at /media/nvidia/rosbags
5. Make sure correct path to Arduino is set in params file

