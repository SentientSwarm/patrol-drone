from setuptools import find_packages, setup

package_name = "patrol_bringup"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        # 02-mission-control (M3+) appends ("share/<pkg>/launch", [...]) here.
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Egemen Cankaya",
    maintainer_email="egemencankaya14@gmail.com",
    description="Launch files, configs, and params for the patrol-drone mission stack (M2 shell).",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            # 02-mission-control (M3+) lands mission node entry points here.
        ],
    },
)
