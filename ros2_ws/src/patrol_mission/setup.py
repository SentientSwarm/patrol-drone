from setuptools import find_packages, setup

package_name = "patrol_mission"

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
    description="Mission orchestration (MissionStateMachine + PatrolMissionNode) for the patrol-drone stack.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "patrol_mission = patrol_mission.node:main",
        ],
    },
)
