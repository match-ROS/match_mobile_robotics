#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Setup file for scene_builder Python modules.
"""

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

setup_args = generate_distutils_setup(
    packages=['scene_control'],
    package_dir={'': 'scripts'},
)

setup(**setup_args)

