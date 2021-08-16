#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

setup_args = generate_distutils_setup(
     packages=['tubular_toolpath_creator'],
     package_dir={'': 'include'},
     # scripts={
     #      'src/tubular_toolpath_node.py',
     #      'include/tubular_toolpath_creator/gap_filter.py', 
     #      'include/tubular_toolpath_creator/tubular_toolpath_server.py',
     #      'include/tubular_toolpath_creator/utils.py'},
     install_requires=['vtk', 'pyvista', 'numpy', 'open3d']
)

#setup(**setup_args)
setup(**setup_args)
