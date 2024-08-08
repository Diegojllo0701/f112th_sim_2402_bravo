from setuptools import find_packages
from setuptools import setup

setup(
    name='f112th_sim_2402_bravo',
    version='0.0.0',
    packages=find_packages(
        include=('f112th_sim_2402_bravo', 'f112th_sim_2402_bravo.*')),
)
