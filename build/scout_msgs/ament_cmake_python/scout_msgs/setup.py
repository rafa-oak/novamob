from setuptools import find_packages
from setuptools import setup

setup(
    name='scout_msgs',
    version='0.1.0',
    packages=find_packages(
        include=('scout_msgs', 'scout_msgs.*')),
)
