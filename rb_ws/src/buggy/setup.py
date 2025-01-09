from glob       import glob
import os

from setuptools import find_packages, setup

package_name = 'buggy'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join("share", package_name), glob("launch/*.xml"))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'hello_world = buggy.hello_world:main',
            'sim_single = buggy.simulator.engine:main',
            'controller = buggy.controller.controller_node:main',
            'buggy_state_converter = buggy.buggy_state_converter:main',
            'watchdog = buggy.watchdog.watchdog_node:main',
        ],
    },
)