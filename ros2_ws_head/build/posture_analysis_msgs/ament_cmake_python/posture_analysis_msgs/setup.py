from setuptools import find_packages
from setuptools import setup

setup(
    name='posture_analysis_msgs',
    version='0.0.0',
    packages=find_packages(
        include=('posture_analysis_msgs', 'posture_analysis_msgs.*')),
)
