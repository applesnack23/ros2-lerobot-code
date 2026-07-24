import os
from glob import glob

from setuptools import find_packages, setup


package_name = "so_arm101_description"


setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),

    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/" + package_name],
        ),

        (
            "share/" + package_name,
            ["package.xml"],
        ),

        (
            os.path.join(
                "share",
                package_name,
                "launch",
            ),
            glob("launch/*.launch.py"),
        ),

        (
            os.path.join(
                "share",
                package_name,
                "urdf",
            ),
            glob("urdf/*.urdf")
            + glob("urdf/*.xacro"),
        ),

        (
            os.path.join(
                "share",
                package_name,
                "urdf",
                "assets",
            ),
            glob("urdf/assets/*"),
        ),

        (
            os.path.join(
                "share",
                package_name,
                "config",
            ),
            glob("config/*"),
        ),
    ],

    install_requires=[
        "setuptools",
    ],

    zip_safe=True,

    maintainer="twiniex",
    maintainer_email="twiniex@todo.todo",

    description=(
        "SO-ARM101 robot description package"
    ),

    license="Apache-2.0",

    extras_require={
        "test": [
            "pytest",
        ],
    },

    entry_points={
        "console_scripts": [],
    },
)