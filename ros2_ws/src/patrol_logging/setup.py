from setuptools import find_packages, setup

package_name = "patrol_logging"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", ["launch/record.launch.py"]),
        ("share/" + package_name + "/config", ["config/recorded_topics.yaml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Egemen Cankaya",
    maintainer_email="egemencankaya14@gmail.com",
    description="Automatic per-run MCAP bag recording (record side of docset 05) for the "
    "patrol-drone stack.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [],
    },
)
