# ASCO Aerial Autonomy


## Setup
Run the setup script in setup/setup.sh to configure Git hooks.  

## Style
This repository uses clang-format for style checking.  Pre-commit hooks ensure that all staged files conform to the style conventions.
To skip pre-commit hooks and force a commit, use `git commit -n`. 

## Dependencies
Install the latest ROS class\_loader package from https://github.com/ros/class_loader.git

## Design Docs
https://paper.dropbox.com/doc/QuadcopterGUI-Framework-Final-Version-ylwUlxLOOPpNM91LsXJyI#:uid=019653862866586&h2=RobotSystem

## Running Tests:
To build and run tests use `catkin build aerial_autonomy --catkin-make-args run_tests`. Output of individual tests can be checked using `rosrun aerial_autonomy test_name`.
To see all test outputs run `catkin run_tests --this`.

## Documentation Coverage
Use coverxygen to generate documentation coverage for the doc: https://github.com/psycofdj/coverxygen
Use the script `scripts/generate_documentation_coverage.bash` to generate documentation into `documentation_coverage_info` folder.
Check the html page in `documentation_coverage_info/index.html` to verify the documentation coverage of the code.

Documentation coverage is also added as a `pre-push` hook. This verifies that 99% of the code is covered with documentation before pushing to remote. It can be skipped for branches with their name starting with `develop*` and also by using `git push --no-verify` command.
