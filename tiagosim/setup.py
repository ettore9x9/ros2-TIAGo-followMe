from setuptools import setup

package_name = 'tiagosim'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
    
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
            
        ('share/' + package_name, ['package.xml']),
        
        ('share/' + package_name + '/launch', ['launch/my_robot_launch.py']),
        
        ('share/' + package_name + '/resource', ['resource/default.rviz']), 
        
        ('share/' + package_name + '/resource', ['resource/map.pgm']),
        
        ('share/' + package_name + '/resource', ['resource/map.yaml']),
        
        ('share/' + package_name + '/resource', ['resource/ros2_control.yml']),
        
        ('share/' + package_name + '/resource', ['resource/tiagosim']),
        
        ('share/' + package_name + '/resource', ['resource/tiago_webots.urdf']),
        
        ('share/' + package_name + '/worlds', ['worlds/world2.wbt']),
        
        
        ('share/' + package_name + '/controllers', ['controllers/my_controller/my_controller.py']),
    ],
    
    
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='fra',
    maintainer_email='francescopagano1999@outlook.it',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
