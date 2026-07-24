from setuptools import find_packages, setup


package_name = "so_arm101_pick_place"


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
    ],
    package_data={
        "": ["py.typed"],
    },
    install_requires=[
        "setuptools",
    ],
    zip_safe=True,
    maintainer="twiniex",
    maintainer_email="twiniex@todo.todo",
    description="SO-ARM101 Pick and Place package",
    license="TODO: License declaration",
    extras_require={
        "test": [
            "pytest",
        ],
    },
    entry_points={
        "console_scripts": [
            (
                "pick_place_sequence_node = "
                "so_arm101_pick_place."
                "pick_place_sequence_node:main"
            ),
        ],
    },
)