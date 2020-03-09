"""A setuptools based setup module.

See:
https://packaging.python.org/en/latest/distributing.html
https://github.com/pypa/sampleproject
"""

# Always prefer setuptools over distutils
from setuptools import setup, find_packages
# To use a consistent encoding
from codecs import open
from os import path

# stripping py files https://stackoverflow.com/a/29718656/135585

try:
        from setuptools.command.build_py import build_py
except ImportError:
        from distutils.command.build_py import build_py

import os
import py_compile


class custom_build_pyc(build_py):
    def byte_compile(self, files):
        for file in files:
            if file.endswith('.py'):
                py_compile.compile(file)
                os.unlink(file)


here = path.abspath(path.dirname(__file__))

# Get the long description from the README file
with open(path.join(here, 'README.md'), encoding='utf-8') as f:
    long_description = f.read()


setup(
    name='rapport',  

    version='0.0.3', 
    
    description='Probabilistic planning for robot teams',  

    package_dir = {'': 'src'},

    packages = find_packages('src'),

    cmdclass = dict(build_py=custom_build_pyc),


)
