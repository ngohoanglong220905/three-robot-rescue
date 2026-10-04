from setuptools import setup

package_name = "multi_robot_formation"

setup(
    name=package_name,
    version="0.0.1",
    packages=[package_name],
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/" + package_name]
        ),
        (
            "share/" + package_name,
            ["package.xml"]
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="long",
    maintainer_email="long@example.com",
    description="A* + Virtual Structure + PID for 3 omni robots",
    license="Apache License 2.0",
    entry_points={
        "console_scripts": [
            "virtual_structure_node = "
            "multi_robot_formation.virtual_structure_node:main",

            "formation_pid_node = "
            "multi_robot_formation.formation_pid_node:main",
        ],
    },
)