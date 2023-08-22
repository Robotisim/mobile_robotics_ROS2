from setuptools import find_packages
from setuptools import setup

setup(
    name='my_custom_message',
    version='0.0.0',
    packages=find_packages(
        include=('my_custom_message', 'my_custom_message.*')),
)
