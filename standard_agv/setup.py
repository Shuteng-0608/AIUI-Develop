from setuptools import setup, find_packages

setup(
    name="sr-modbus-sdk",
    version="0.1.0",
    description="SR Modbus SDK for Python",
    author="Your Name",
    author_email="your@email.com",
    url="https://github.com/yourname/sr-modbus-sdk-py",
    packages=find_packages(where="src"),  # 关键：指定src目录下的包
    package_dir={"": "src"},  # 关键：设置源码目录为src
    classifiers=[
        "Programming Language :: Python :: 3",
        "License :: OSI Approved :: MIT License",
        "Operating System :: OS Independent",
    ],
    python_requires='>=3.6',
)