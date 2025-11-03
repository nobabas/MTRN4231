from setuptools import find_packages
from setuptools import setup

setup(
    name='blue_detector_msgs',
    version='0.0.0',
    packages=find_packages(
        include=('blue_detector_msgs', 'blue_detector_msgs.*')),
)
