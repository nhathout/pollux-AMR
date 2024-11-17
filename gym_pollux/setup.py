from setuptools import setup, find_packages

setup(
    name='gym_pollux',
    version='0.0.1',
    description='Custom Gym environment for pollux-AMR',
    author='Your Name',
    packages=find_packages(),
    install_requires=['gym>=0.21.0', 'numpy'],
)
