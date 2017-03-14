# ASCO Aerial Autonomy

## Introduction
The doxygen documentation to the project can be found [here](https://jhu-asco.github.io/aerial_autonomy/).


## Setup
Run the setup script in setup/setup.sh to configure Git hooks.  

Install the following dependencies (lcov, protobuf, doxygen, doxypy, coverxygen, google-glog, class-loader). On Ubuntu 14.04 run the following line in a terminal

    sudo apt-get install lcov protobuf-compiler libprotobuf-dev doxygen doxypy libgoogle-glog-dev ros-indigo-class-loader
    sudo pip install coverxygen

## Running Executables
The package provides an `aerial_autonomy_node` executable which loads a state machine and hardware and waits for event commands from a ROS topic. The `rqt_aerial_autonomy_gui` script
provides a GUI to generate events for the state machine. The rqt plugin can be loaded along with `rqt_rviz` in the `rqt_gui` framework.

The `simulator.launch` file in the launch folder executes the state machine node using simulated hardware. The GUI can be launched individually using rosrun. The steps to launch a simulated quadrotor with the state machine are

    roslaunch aerial_autonomy simulator.launch
    rosrun aerial_autonomy rqt_aerial_autonomy_gui  # In a separate tab

## Running Tests
To build and run tests use `catkin build aerial_autonomy --catkin-make-args run_tests`. Output of individual tests can be checked using `rosrun aerial_autonomy test_name`.
To see all test outputs run `catkin run_tests --this`.

## Logging
GLOG is used to log messages from the state machine. The messages are divided into different levels (INFO, WARNING, ERROR, etc.,). The information messages are divided into different verbosity levels (0,1,2 and so on). The verbosity level can be adjusted using the environment variable `GLOG_v`. If the environment variable is set to 1 (`export GLOG_v=1`), then all the messages with verbosity 0 and 1 are streamed to stderr output.

The log messages are also recorded into log files in the `logs` folder. The symbolic links `aerial_autonomy_node.INFO` and `aerial_autonomy_node.WARNING` in the log folder point to the latest log files. The log directory can be changed using the `GLOG_log_dir` environment variable. More information about the log files can be found in the Google Log [documentation](http://rpg.ifi.uzh.ch/docs/glog.html).

The `simulator` launch file introduced above allows for specifying the log level and log directory using roslaunch arguments `log_level` and `log_dir` respectively. For example

    roslaunch aerial_autonomy simulator.launch log_level:=1  # Prints all the verbose log messages with priority 0 and 1.

## Style
This repository uses clang-format for style checking.  Pre-commit hooks ensure that all staged files conform to the style conventions.
To skip pre-commit hooks and force a commit, use `git commit -n`. 

## Documentation Coverage
We use coverxygen to generate documentation coverage for the doc: https://github.com/psycofdj/coverxygen

Use the script `scripts/generate_documentation_coverage.bash` to generate documentation into `documentation_coverage_info` folder.
Check the html page in `documentation_coverage_info/index.html` to verify the documentation coverage of the code.

Documentation coverage is also added as a `pre-push` hook. This verifies that 95% of the code is covered with documentation before pushing to remote. It can be skipped for branches with their name starting with `develop*` and also by using `git push --no-verify` command.

## Test Coverage
We use `lcov` to generate the test coverage report into the `test_coverage_info` folder. The script `scripts/generate_test_coverage.bash` is used to run tests in the project and generate test coverage report into `test_coverage_info` folder. The script is generated using CMake. Run `catkin build aerial_autonomy` to create the script. Check the html page `test_coverage_info/index.html` to check the line and function coverage. The bash script is generated
by running CMake using `catkin build aerial_autonomy`.

The test generation is integrated into the `pre-push` commit hook. This runs the above test coverage generation script and verifies that the coverage level is above 95% threshold. This can be skipped by either naming the branch as `develop[your_branch_name]` or using `git push --no-verify`.

## Uploading documentation
The documentation is uploaded through `gh-pages` branch. The docs are created in master and passed to the `gh-pages` branch using `scripts/applydocs.bash` script. The script checks that there are not uncommited changes before uploading documentation to avoid issues with git. The script also requires that you explicitly link gh-pages branch to the remote using `git branch --set-upstream-to=[GH_PAGES_REMOTE]`
