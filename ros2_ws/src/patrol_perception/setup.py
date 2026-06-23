from setuptools import find_packages, setup

package_name = "patrol_perception"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Egemen Cankaya",
    maintainer_email="egemencankaya14@gmail.com",
    description="Perception & checkpoint capture (CheckpointCapture contract + capture node) for the patrol-drone stack.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "perception_node = patrol_perception.perception_node:main",
        ],
    },
)
