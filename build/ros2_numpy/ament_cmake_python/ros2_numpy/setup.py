from setuptools import find_packages
from setuptools import setup

setup(
    name='ros2_numpy',
    version='2.0.12',
    packages=find_packages(
        include=('ros2_numpy', 'ros2_numpy.*')),
)
