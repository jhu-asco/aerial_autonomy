# ASCO Aerial Autonomy

## Introduction
The doxygen documentation to the project can be found [here](https://jhu-asco.github.io/aerial_autonomy/).


## Setup
Run the setup script in scripts/setup/setup.sh to configure Git hooks.  

Install the following dependencies (lcov, protobuf, doxygen, doxypy, coverxygen, google-glog, class-loader). On Ubuntu 18.04 run the following in a terminal (for different versions of Ubuntu replace melodic with your ROS version)

    sudo apt-get install lcov protobuf-compiler libprotobuf-dev doxygen doxypy libgoogle-glog-dev ros-melodic-class-loader ros-melodic-ar-track-alvar-msgs ros-melodic-vision-msgs autoconf python-pip ros-melodic-serial ros-melodic-map-server libarmadillo-dev
    sudo pip install coverxygen

Install protobuf 3.1: (Alternatively, protobuf 3.0.0, which is default with ROS Melodic, can be used and these steps can be skipped. Check version with `protoc --version`)

    git clone https://github.com/google/protobuf.git
    cd protobuf
    git checkout v3.1.0
    ./autogen.sh
    ./configure
    make
    sudo make install
    sudo ldconfig

Install googletest `release 1.8.0`. This version fixes a bug with `ASSERT_TRUE` as explained [here](https://github.com/google/googletest/issues/429). To install googletest, follow these steps

    git  clone https://github.com/google/googletest.git
    cd googletest
    git checkout release-1.8.0
    mkdir build
    cd build
    cmake .. -DCMAKE_BUILD_TYPE=Release -DBUILD_SHARED_LIBS=ON -DBUILD_GMOCK=ON -DBUILD_GTEST=ON
    make
    sudo make install
    sudo ldconfig

Install OpenCV with OpenCV Contrib (version must include tracking module). Follow the steps for installing from source [here](https://linuxize.com/post/how-to-install-opencv-on-ubuntu-18-04/) to install from source on Ubuntu 18.04. If a version of OpenCV is already installed on your system you may want to install that version from source. Note: Source code for OpenCV 3.2.0 has an extra else statement on line 21 of cmake/OpenCVCompilerOptions.cmake. This block needs to be removed. The following can be used to check if your system currently has a version of OpenCV installed:

    pkg-config --modversion opencv
	python3 -c "import cv2; print(cv2.version)"
	python2 -c "import cv2; print(cv2.version)"

Install our GCOP (Geometric Control, Optimization, and Planning) package. Build with support for casadi (USE_CASADI) and install the dependences from the GCOP README. Do the following **after required and optional dependencies from the GCOP README have been installed (Numbers 5 and 6)**:

    git clone https://github.com/jhu-asco/gcop.git
    cd gcop
    mkdir build
    cd build
    cmake -DUSE_CASADI=ON ..
    sudo make install

Create a ROS workspace. Run the following in your ROS workspace src folder to setup UAV hardware drivers

    git clone -b hydro-devel https://github.com/jhu-asco/quadcopter_parsers.git
    git clone -b 3.2.3 https://github.com/jhu-asco/Onboard-SDK-ROS.git

Install gcop_comm for trajectory visualization (other packages in the repo can be ignored) in the ROS workspace src folder

    git clone -b hydro-devel https://github.com/jhu-asco/gcop_ros_packages.git

### Optional: Manipulator packages
Optionally, to install drivers related to aerial manipulation, run the following in your ROS src folder

    git clone https://git.lcsr.jhu.edu/mshecke1/arm_plugins.git
    git clone https://git.lcsr.jhu.edu/ggarime1/controllers.git
    git clone https://git.lcsr.jhu.edu/ggarime1/dynamixelsdk.git

## Build
This package can be cloned into the same ROS workspace src folder and built with `catkin build`. Be sure to source the workspace's `devel/setup.bash`.

### Arm Plugins
Building with arm plugins can be turned off by setting the `USE_ARM_PLUGINS` cmake argument to OFF

    catkin build -DUSE_ARM_PLUGINS=OFF

This is recommended, when arm plugins are not needed, for the code to compile faster and using less system resources.

## Running Executables
The package provides a `uav_system_node` executable which loads a state machine and hardware and waits for event commands from a ROS topic. The `rqt_aerial_autonomy_gui` script
provides a GUI to generate events for the state machine. The rqt plugin can be loaded along with `rqt_rviz` in the `rqt_gui` framework.

The `simulator.launch` file in the launch folder executes the state machine node using simulated hardware. The GUI can be launched individually using rosrun, or with `simulator_rqt_aerial_autonomy.launch`. The steps to launch a simulated quadrotor with the state machine are

    roslaunch aerial_autonomy simulator.launch
    roslaunch aerial_autonomy simulator_rqt_aerial_autonomy.launch  # In a separate tab

## Running Tests
To build and run tests use `catkin build aerial_autonomy --catkin-make-args run_tests`. Output of individual tests can be checked using `rosrun aerial_autonomy test_name`.
To see all test outputs run `catkin run_tests --this`.

## Logging
GLOG is used to log messages from the state machine. The messages are divided into different levels (INFO, WARNING, ERROR, etc.,). The information messages are divided into different verbosity levels (0,1,2 and so on). The verbosity level can be adjusted using the environment variable `GLOG_v`. If the environment variable is set to 1 (`export GLOG_v=1`), then all the messages with verbosity 0 and 1 are streamed to stderr output.

The log messages are also recorded into log files in the `logs` folder. The symbolic links `uav_system_node.INFO` and `uav_system_node.WARNING` in the log folder point to the latest log files. The log directory can be changed using the `GLOG_log_dir` environment variable. More information about the log files can be found in the Google Log [documentation](http://rpg.ifi.uzh.ch/docs/glog.html).

The `simulator` launch file introduced above allows for specifying the log level and log directory using roslaunch arguments `log_level` and `log_dir` respectively. For example

    roslaunch aerial_autonomy simulator.launch log_level:=1  # Prints all the verbose log messages with priority 0 and 1.

## Style
This repository uses clang-format for style checking.  Pre-commit hooks ensure that all staged files conform to the style conventions.
To skip pre-commit hooks and force a commit, use `git commit -n`.

## Documentation Coverage
We use coverxygen to generate documentation coverage for the doc: https://github.com/psycofdj/coverxygen

Use the script `scripts/generate_documentation_coverage.bash` to generate documentation into `.documentation_coverage_info` folder.
Check the html page in `.documentation_coverage_info/index.html` to verify the documentation coverage of the code.

Documentation coverage is also added as a `pre-push` hook. This verifies that 95% of the code is covered with documentation before pushing to remote. It can be skipped for branches with their name starting with `develop*` and also by using `git push --no-verify` command.

## Test Coverage
We use `lcov` to generate the test coverage report into the `.test_coverage_info` folder. The script `scripts/generate_test_coverage.bash` is used to run tests in the project and generate test coverage report into `.test_coverage_info` folder. The script is generated using CMake. Run `catkin build aerial_autonomy` to create the script. Check the html page `.test_coverage_info/index.html` to check the line and function coverage. The bash script is generated
by running CMake using `catkin build aerial_autonomy`.

The test generation is integrated into the `pre-push` commit hook. This runs the above test coverage generation script and verifies that the coverage level is above 95% threshold. This can be skipped by either naming the branch as `develop[your_branch_name]` or using `git push --no-verify`.

## Uploading documentation
The documentation is uploaded through `gh-pages` branch. The docs are created in master and passed to the `gh-pages` branch using `scripts/applydocs.bash` script. The script checks that there are not uncommited changes before uploading documentation to avoid issues with git. The script also requires that you explicitly link gh-pages branch to the remote using `git branch --set-upstream-to=[GH_PAGES_REMOTE]`

## Generating Visual graphs from state machines
The script `scripts/generate_dot_files.py` converts the transition tables in state machines to dot format and also png format. The script automatically runs through all the state machines stored in the `include/aerial_autonomy/state_machines` folder.

    Usage: ./generate_dot_files.py

## Hand-eye Calibration
This section describes how to automatically calibrate a transform from a camera to an arm.

### Data Collection
Attach an AR tag to the end effector of your arm.

Use `rosbag record /ar_pose_marker /your_end_effector_position` where `/your_end_effector_position` is published by your arm driver and gives the position of the end effector in the arm frame (probably based on forward kinematics).

Launch ar_track_alvar and move the arm around so that the end effector AR tag is visible in the camera.

Extract the AR marker data from the bag file to a csv: `rostopic echo -b your_data.bag -p --nostr /ar_pose_marker > marker_data.csv`

### Calibration script
Use the matlab script `scripts/calib/arm_camera_calib.m` along with your_data.bag and marker_data.csv to generate a calibrated transformation.
It uses non-linear least squares to find the hand-eye transformation.
