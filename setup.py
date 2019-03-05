##########################  FWMAV Simulation  #########################
# Version 0.3
# Fan Fei		Feb 2019
# Direct motor driven flapping wing MAV simulation
#######################################################################

from setuptools import setup
from distutils.core import Extension
from sys import platform as _platform

CXX_FLAGS = ''
if _platform == "darwin":
	CXX_FLAGS += '-mmacosx-version-min=10.9 '

Wing_module = Extension('_Wing',
						sources = ['flappy/envs/Wing_wrap.cxx', 'flappy/envs/Wing.cpp'],
						extra_compile_args=CXX_FLAGS.split())

setup (name = 'Wing',
		version = '0.1',
		author = 	"Fan Fei",
		description = "Flapping wing with aero model",
		install_requires=['gym', 'stable_baselines', 'numpy'],
		ext_modules = [Wing_module],
		py_modules = ["Wing"])