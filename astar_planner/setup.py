from setuptools import find_packages, setup

package_name = 'astar_planner'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='long',
    maintainer_email='long@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
   entry_points={
    'console_scripts': [
        'astar_node = astar_planner.astar_node:main',
        'pid_follower = astar_planner.pid_follower:main',
        'three_robot_dstar_mpc = astar_planner.three_robot_dstar_mpc:main',
        'robot_distributed_mpc = astar_planner.three_robot_dstar_mpc:robot_mpc_main',
    ],
},
)
