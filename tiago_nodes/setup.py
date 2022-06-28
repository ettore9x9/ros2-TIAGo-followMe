from setuptools import setup

package_name = 'tiago_nodes'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/tiago_launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ettore',
    maintainer_email='ettoresani0@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'depth_finder = tiago_nodes.depth_finder:main',
            'pid_controller = tiago_nodes.pid_controller:main',
        ],
    },
)
