#!/usr/bin/python
## ! DO NOT MANUALLY INVOKE THIS setup.py, USE CATKIN INSTEAD

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

setup_args = generate_distutils_setup(
    packages=['behaviour_components', 'rqt_planner_gui'],
    package_dir={'': 'src'},
    scripts=['scripts/rqt_planner_gui']
)

setup(**setup_args)
