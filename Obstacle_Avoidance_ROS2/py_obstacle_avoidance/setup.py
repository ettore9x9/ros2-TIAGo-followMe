import os
from setuptools import setup
from glob import glob

package_name = 'py_obstacle_avoidance'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='brmil07',
    maintainer_email='To-Do',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
          'robot_controller = py_obstacle_avoidance.robot_controller:main',
          'robot_estimator = py_obstacle_avoidance.robot_estimator:main'
        ],
    },
)
