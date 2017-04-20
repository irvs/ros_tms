#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

setup_args = generate_distutils_setup(
    packages=['tms_ur_speaker'],
    package_dir={'': 'src'},
)

setup(**setup_args)
