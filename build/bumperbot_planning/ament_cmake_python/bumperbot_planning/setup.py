from setuptools import find_packages
from setuptools import setup

setup(
    name='bumperbot_planning',
    version='0.0.0',
    packages=find_packages(
        include=('bumperbot_planning', 'bumperbot_planning.*')),
)
