#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

setup_args = generate_distutils_setup(
     packages=['tubular_toolpath_creator'],
     package_dir={'': 'src'},
     scripts={'src/gap_filter.py', 
          'src/tubular_toolpath_server.py',
          'src/tubular_toolpath_node.py',
          'src/utils.py'},
     install_requires=['vtk', 'pyvista', 'numpy', 'open3d']
)

#setup(**setup_args)
setup(**setup_args)
