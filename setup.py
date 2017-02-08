#!/usr/bin/env python
"""
Script to install aerial_autonomy package.
http://docs.ros.org/jade/api/catkin/html/user_guide/setup_dot_py.html
"""

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(packages=['aerial_autonomy'],
                             package_dir={'': 'src'}
                             )

setup(**d)
