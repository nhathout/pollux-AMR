from setuptools import setup, find_packages

setup(
    name='gym_pollux',
    version='0.0.1',
    description='Custom Gymnasium environment for pollux-AMR',
    author='Noah Hathout',
    packages=find_packages(),
    install_requires=['gymnasium', 'numpy'],
)
