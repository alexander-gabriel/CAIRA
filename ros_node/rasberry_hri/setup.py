#!/usr/bin/env python
# ----------------------------------
# @author: einalex
# @email: agabriel@lincoln.ac.uk
# @date: 27/05/2019
# ----------------------------------

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    # don't do this unless you want a globally visible script
    # scripts=['bin/myscript'],
    packages=['skeleton_pipeline', 'robco', 'bdi', 'probcog', 'experiments', 'common'],
    package_dir={'': 'src'}
)

setup(**d)
