# ASCO Aerial Autonomy


## Setup
Run the setup script in setup/setup.sh to configure Git hooks.  

## Style
This repository uses clang-format for style checking.  Pre-commit hooks ensure that all staged files conform to the style conventions.
To skip pre-commit hooks and force a commit, use `git commit -n`. 

## Dependencies
Install the latest ROS class_loader package from https://github.com/ros/class_loader.git

## Design Docs
https://paper.dropbox.com/doc/QuadcopterGUI-Framework-Final-Version-ylwUlxLOOPpNM91LsXJyI#:uid=019653862866586&h2=RobotSystem

## Running Tests:
To build and run tests use `catkin build aerial_autonomy --catkin-make-args run_tests`. Output of individual tests can be checked using `rosrun aerial_autonomy test_name`.
