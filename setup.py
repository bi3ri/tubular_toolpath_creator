#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

setup_args = generate_distutils_setup(
     packages=['tubular_toolpath_creator'],
     package_dir={'': 'src'},
     install_requires=['vtk', 'pyvista', 'numpy', 'open3d']
)

setup(**setup_args)scriptsscripts