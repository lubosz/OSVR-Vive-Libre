#!/usr/bin/python

from distutils.core import setup
from distutils.extension import Extension
from Cython.Distutils import build_ext
 
module1 = Extension("pyvive", 
    ["pyvive.pyx", "vl_python.cpp"],
    language="c++",
    libraries=["vive_libre"],
    include_dirs=['../src'])
 
setup(name = 'pyive',
    version = '1.0',
    description = 'Python Vive Libre Wrapper',
    ext_modules=[module1],
    cmdclass = {'build_ext': build_ext})
