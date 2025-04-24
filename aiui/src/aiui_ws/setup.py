from setuptools import setup, find_packages
from catkin_pkg.python_setup import generate_distutils_setup

setup_args = generate_distutils_setup(
    packages=find_packages('lib'),  # 关键修改
    #packages=['processStrategy'],  # 替换为你的库名
    package_dir={'': 'lib'},
)



setup(**setup_args)